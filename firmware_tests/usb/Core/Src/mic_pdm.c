#include "mic_pdm.h"

#include <string.h>
#include <math.h>

#include "main.h"
#include "stm32wbxx_hal_sai_ex.h"

/*
 * Short microphone capture helper.
 * CubeMX creates the base SAI/DMA handles, but this module temporarily
 * reconfigures them and the shared GPIO pins only for the capture window
 * around a dose event. IRQ callbacks only note buffer progress; heavy work
 * such as sample statistics runs later in mic_pdm_task().
 */

/* Size of the raw DMA buffer in 32-bit PDM words. */
#define MIC_PDM_RAW_BUFFER_WORDS      4096u
/* Raw words processed per DMA half-complete callback. */
#define MIC_PDM_HALF_BUFFER_WORDS     (MIC_PDM_RAW_BUFFER_WORDS / 2u)
/* Post-stop tail time before capture is finalized. */
#define MIC_PDM_STOP_TAIL_MS          32u
/* Hard safety limit for any single capture; force-finalize if exceeded. */
#define MIC_PDM_MAX_CAPTURE_MS        600000u
/* CIC decimation filter: order 4, R=32.
 * With 8 raw PDM bits per DMA word (pair2-L byte) at ~187.5 kHz frame rate
 * the resulting PCM sample rate is 187500 × 8 / 32 ≈ 46.875 kHz.
 * R=32 gives ~24 dB better SQNR than R=16 (75 dB vs 51 dB).
 */
#define MIC_PDM_CIC_ORDER             4u
#define MIC_PDM_CIC_DECIMATION        32u
/* IM67D120 needs ~25 ms after clock restart before PDM output stabilises.
 * Skip this many DMA words at the start of each capture to avoid wakeup
 * transients polluting the peak/avg metrics.
 * At ~187.5 kHz frame rate: 4800 words ≈ 25.6 ms.
 */
#define MIC_PDM_WAKEUP_SKIP_WORDS    4800u
/* Number of decimated CIC output samples to discard after the wakeup skip.
 * The CIC comb cascade starts from zero delay registers; the first ORDER
 * output samples are dominated by startup transient, not real audio.
 * 20 samples ≈ 0.4 ms at 46.875 kHz — negligible cost.
 */
#define MIC_PDM_CIC_SETTLE_SAMPLES   20u
/* Goertzel single-frequency detector for ~15.4 kHz click detection.
 * N=64 samples per bin at 46.875 kHz → ~1.365 ms per bin.
 * k=21 → detected frequency = 21 * 46875 / 64 ≈ 15381 Hz.
 * coeff = 2*cos(2*pi*21/64).
 */
#define GOERTZEL_N          64u
#define GOERTZEL_COEFF      (-0.94280904f)
/* Frequency scan: run parallel Goertzel detectors across bins k=1..31
 * to find where the pen click energy is concentrated.
 * Set to 0 for normal single-bin detection mode.
 */
#define GOERTZEL_SCAN_MODE     0u
#if GOERTZEL_SCAN_MODE
#define GOERTZEL_SCAN_K_MIN    2u
#define GOERTZEL_SCAN_K_MAX    32u
#define GOERTZEL_SCAN_K_STEP   2u
#define GOERTZEL_SCAN_BINS     (((GOERTZEL_SCAN_K_MAX - GOERTZEL_SCAN_K_MIN) / GOERTZEL_SCAN_K_STEP) + 1u)
#endif

/* Broadband spike detector: energy in short windows vs running background. */
#define SPIKE_DETECT_MODE      1u
#if SPIKE_DETECT_MODE
#define SPIKE_WIN_SIZE         256u  /* samples per window (~5.46 ms at 46.875 kHz) */
#define SPIKE_DETECT_FACTOR    3.0f  /* peak must exceed this × bg to count */
#define SPIKE_REARM_FACTOR     1.5f  /* EMA must fall below this × bg to re-arm */
/* After a counted peak the EMA will decay.  Re-arm requires two conditions:
 * 1) EMA must fall below REARM_FACTOR×bg (hysteresis band: 1.5× – 3×)
 * 2) EMA must then start rising (direction guard against wobble)
 */
#endif
/* Accumulated FFT: 32 Goertzel bins running alongside spike detection.
 * N=4096 samples per block at 46.875 kHz → 11.44 Hz resolution, ~87 ms/block.
 * Peak magnitude per bin is tracked across all blocks for the full capture.
 * 32 bins: log-spaced 23 Hz–5.7 kHz (bins 0–15), linear 6.9–22.9 kHz (bins 16–31).
 * Per-sample cost: 32 Goertzel updates ≈ 640 cycles at -O0 (budget ~1365).
 */
#define ACCUM_FFT_ENABLE       1u
#if ACCUM_FFT_ENABLE
#define ACCUM_FFT_N            4096u
#define ACCUM_FFT_BINS         32u
#endif

/* Debug option: keep the microphone power rail enabled even when idle. */
#define MIC_PDM_DEBUG_ALWAYS_POWERED  1u

typedef enum
{
  MIC_PDM_STATE_IDLE = 0,
  MIC_PDM_STATE_RUNNING,
  MIC_PDM_STATE_STOPPING
} mic_pdm_state_t;

extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef hdma_sai1_a;

/* Shared DMA buffer and state exchanged between callbacks and the main loop. */
static uint32_t s_raw_buffer[MIC_PDM_RAW_BUFFER_WORDS];
static volatile uint8_t s_pending_halves = 0u;
static volatile uint8_t s_overflow = 0u;
static volatile uint8_t s_dma_error = 0u;
static uint32_t s_start_tick_ms = 0u;
static uint32_t s_stop_after_ms = 0u;
static uint64_t s_total_abs_sum = 0u;
static int64_t s_total_signed_sum = 0;
static mic_pdm_state_t s_state = MIC_PDM_STATE_IDLE;
static bool s_initialized = false;
static bool s_result_ready = false;
static bool s_current_ckstr = false;
static uint32_t s_wakeup_skip = 0u;
static uint32_t s_cic_int[MIC_PDM_CIC_ORDER];
static uint32_t s_cic_comb[MIC_PDM_CIC_ORDER];
static uint8_t s_cic_count = 0u;
static uint32_t s_cic_settle = 0u;    /* decimated output samples still to discard */
static int32_t s_bg_offset = 0;       /* background noise offset from Enter calibration */

/* Byte-at-a-time CIC4 lookup tables.
 * Each table maps a PDM byte (256 entries) to the contribution that byte
 * makes to the corresponding integrator stage after processing 8 bits.
 * This replaces 8 individual bit iterations with one table lookup per stage.
 *
 * PDM encoding: bit=1 → +1, bit=0 → −1 (two's complement 0xFFFFFFFF).
 *
 * Stage 0: sum of 8 signed bits = popcount(byte)*2 - 8.
 * Stage 1..3: derived from combinatorial sums of the running prefix sums.
 *
 * Tables are int16_t (fits all values) to save RAM: 4 × 256 × 2 = 2 KB.
 */
static int16_t s_cic_lut0[256];
static int16_t s_cic_lut1[256];
static int16_t s_cic_lut2[256];
static int16_t s_cic_lut3[256];
static bool s_cic_luts_ready = false;
/* Goertzel state */
static float s_gtz_s1 = 0.0f;
static float s_gtz_s2 = 0.0f;
static uint32_t s_gtz_n = 0u;
static float s_gtz_threshold = 0.0f;
static float s_dc_est = 0.0f;          /* running DC estimate for high-pass */
static bool s_gtz_calibrating = false;
static float s_gtz_cal_mean = 0.0f;
static float s_gtz_cal_m2 = 0.0f;
static uint32_t s_gtz_cal_count = 0u;
#if GOERTZEL_SCAN_MODE
static float s_gtz_scan_s1[GOERTZEL_SCAN_BINS];
static float s_gtz_scan_s2[GOERTZEL_SCAN_BINS];
static float s_gtz_scan_coeff[GOERTZEL_SCAN_BINS];
static float s_gtz_scan_peak[GOERTZEL_SCAN_BINS];
static float s_gtz_scan_bg[GOERTZEL_SCAN_BINS];   /* background avg per bin from calibration */
static bool s_gtz_scan_coeff_ready = false;
#endif
#if SPIKE_DETECT_MODE
static float s_spike_win_energy = 0.0f;   /* accumulator for current window */
static uint32_t s_spike_win_n = 0u;       /* sample count in current window */
static float s_spike_bg = 0.0f;           /* IIR background energy estimate */
static float s_spike_peak = 0.0f;         /* max window energy seen (MES) */
static uint32_t s_spike_count_3x = 0u;    /* windows exceeding 3× background */
static uint32_t s_spike_count_5x = 0u;    /* windows exceeding 5× background */
static bool s_spike_bg_valid = false;      /* set after calibration */
static uint32_t s_spike_peak_count = 0u;   /* detected click peaks */
static bool s_spike_post_peak = false;     /* true = EMA decaying after counted peak, suppress new peaks */
static bool s_spike_fell_below = false;    /* EMA dropped below 3×bg since last counted peak */
static float s_spike_smooth_e = 0.0f;       /* EMA-smoothed window energy for peak detect */
static bool s_spike_was_rising = false;     /* prev smoothed energy was rising above threshold */
static float s_spike_prev_smooth = 0.0f;   /* previous smoothed energy */
static uint16_t s_spike_env_trace[MIC_PDM_ENVELOPE_TRACE_BINS];
static uint32_t s_spike_env_count = 0u;
static bool s_spike_env_ready = false;
#endif
#if ACCUM_FFT_ENABLE
/* k values sorted ascending, log-spaced low band + linear-spaced high band.
 * Frequencies: k * 46875 / 4096 Hz.
 * Bin  0: k=   2 →    23 Hz    Bin 16: k= 500 →  5722 Hz
 * Bin  1: k=   3 →    34 Hz    Bin 17: k= 600 →  6866 Hz
 * Bin  2: k=   5 →    57 Hz    Bin 18: k= 700 →  8008 Hz
 * Bin  3: k=   7 →    80 Hz    Bin 19: k= 800 →  9155 Hz
 * Bin  4: k=  10 →   114 Hz    Bin 20: k= 900 → 10300 Hz
 * Bin  5: k=  15 →   172 Hz    Bin 21: k=1000 → 11444 Hz
 * Bin  6: k=  21 →   240 Hz    Bin 22: k=1100 → 12589 Hz
 * Bin  7: k=  30 →   343 Hz    Bin 23: k=1200 → 13733 Hz
 * Bin  8: k=  43 →   492 Hz    Bin 24: k=1300 → 14878 Hz
 * Bin  9: k=  61 →   698 Hz    Bin 25: k=1400 → 16022 Hz
 * Bin 10: k=  87 →   996 Hz    Bin 26: k=1500 → 17166 Hz
 * Bin 11: k= 123 →  1408 Hz    Bin 27: k=1600 → 18311 Hz
 * Bin 12: k= 175 →  2003 Hz    Bin 28: k=1700 → 19455 Hz
 * Bin 13: k= 248 →  2838 Hz    Bin 29: k=1748 → 20004 Hz
 * Bin 14: k= 352 →  4028 Hz    Bin 30: k=1800 → 20599 Hz
 * Bin 15: k= 500 →  5722 Hz    Bin 31: k=1900 → 21744 Hz
 */
static const uint16_t s_afft_k[ACCUM_FFT_BINS] = {
    2,   3,   5,   7,  10,  15,  21,  30,
   43,  61,  87, 123, 175, 248, 352, 500,
  600, 700, 800, 900,1000,1100,1200,1300,
 1400,1500,1600,1700,1748,1800,1900,2000
};
static float    s_afft_coeff[ACCUM_FFT_BINS];
static float    s_afft_s1[ACCUM_FFT_BINS];
static float    s_afft_s2[ACCUM_FFT_BINS];
static float    s_afft_peak[ACCUM_FFT_BINS];   /* peak magnitude per bin across all blocks */
static uint32_t s_afft_n = 0u;                 /* sample counter within current block */
static uint32_t s_afft_blocks = 0u;            /* number of completed blocks */
static uint16_t s_fft_mags[ACCUM_FFT_BINS];
static bool     s_afft_coeff_ready = false;
static bool     s_fft_ready = false;
#endif
static mic_pdm_result_t s_result;

static bool mic_pdm_tick_expired(uint32_t deadline_ms);
static void mic_pdm_configure_shared_pins_for_capture(void);
static void mic_pdm_restore_shared_pins(void);
static void mic_pdm_power_on(void);
static void mic_pdm_power_off(void);
static bool mic_pdm_prepare_sai(bool ckstr);
static bool mic_pdm_start_hw(void);
static void mic_pdm_stop_hw(void);
static void mic_pdm_reset_result(void);
static void mic_pdm_update_transport_metrics(uint32_t start_index);
static void mic_pdm_process_half(uint32_t start_index);
static void mic_pdm_finalize(void);

static bool mic_pdm_tick_expired(uint32_t deadline_ms)
{
  return ((int32_t)(HAL_GetTick() - deadline_ms) >= 0);
}

/* Move PA8/PA9/PA10 from idle GPIO mode into the SAI alternate function for active capture. */
static void mic_pdm_configure_shared_pins_for_capture(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Reuse only the SAI data/clock pins; PA3 stays dedicated to TMP102 alert. */
  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF3_SAI1;
  HAL_GPIO_Init(GPIOA, &gpio);
}

static void mic_pdm_power_on(void)
{
  HAL_GPIO_WritePin(Mic_PWR_GPIO_Port, Mic_PWR_Pin, GPIO_PIN_SET);
}

static void mic_pdm_power_off(void)
{
#if (MIC_PDM_DEBUG_ALWAYS_POWERED == 1u)
  HAL_GPIO_WritePin(Mic_PWR_GPIO_Port, Mic_PWR_Pin, GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(Mic_PWR_GPIO_Port, Mic_PWR_Pin, GPIO_PIN_RESET);
#endif
}

/* Return shared microphone pins to a quiet idle state after capture completes or fails. */
static void mic_pdm_restore_shared_pins(void)
{
  GPIO_InitTypeDef gpio = {0};

  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);

  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  mic_pdm_power_off();
}

/* Program the runtime SAI/DMA settings required by the microphone capture path. */
static bool mic_pdm_prepare_sai(bool ckstr)
{
  SAI_HandleTypeDef *hsai = &hsai_BlockA1;
  HAL_StatusTypeDef status;

  if ((hsai->Instance != SAI1_Block_A) || (hsai->hdmarx == NULL) ||
      (hsai->hdmarx->Instance == NULL)) {
    s_result.diag_stage = 1u;
    return false;
  }

  s_result.diag_stage = 2u;
  if (HAL_SAI_GetState(hsai) == HAL_SAI_STATE_BUSY_RX) {
    (void)HAL_SAI_DMAStop(hsai);
  }

  /* Force a clean RESET -> INIT transition on every capture start so HAL runs
   * MspDeInit -> MspInit as a balanced pair, reapplying SAI1 clock tree, DMA
   * and NVIC setup regardless of which state the peripheral was left in.
   * DeInit returns HAL_ERROR when already in RESET state -- safe to ignore.
   */
  (void)HAL_SAI_DeInit(hsai);

  /* Keep the HAL config aligned with the previously working low-level setup. */
  hsai->Init.Protocol = SAI_FREE_PROTOCOL;
  hsai->Init.AudioMode = SAI_MODEMASTER_RX;
  hsai->Init.DataSize = SAI_DATASIZE_8;
  hsai->Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai->Init.ClockStrobing = ckstr ? SAI_CLOCKSTROBING_RISINGEDGE : SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai->Init.Synchro = SAI_ASYNCHRONOUS;
  hsai->Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai->Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai->Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai->Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
  hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai->Init.MonoStereoMode = SAI_STEREOMODE;
  hsai->Init.CompandingMode = SAI_NOCOMPANDING;
  hsai->Init.PdmInit.Activation = ENABLE;
  hsai->Init.PdmInit.MicPairsNbr = 2;
  hsai->Init.PdmInit.ClockEnable = SAI_PDM_CLOCK2_ENABLE;
  hsai->FrameInit.FrameLength = 32;
  hsai->FrameInit.ActiveFrameLength = 1;
  hsai->FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai->FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai->FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai->SlotInit.FirstBitOffset = 0;
  hsai->SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai->SlotInit.SlotNumber = 4;
  hsai->SlotInit.SlotActive = SAI_SLOTACTIVE_2;

  status = HAL_SAI_Init(hsai);
  s_result.sai_init_status = (uint8_t)status;
  s_result.sai_clk_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI1);
  s_result.sai_state = (uint8_t)HAL_SAI_GetState(hsai);
  s_result.sai_error = HAL_SAI_GetError(hsai);
  if (status != HAL_OK) {
    s_result.diag_stage = 3u;
    return false;
  }

  hsai->hdmarx->Init.Mode = DMA_CIRCULAR;
  status = HAL_DMA_Init(hsai->hdmarx);
  s_result.dma_init_status = (uint8_t)status;
  s_result.dma_state = (uint8_t)HAL_DMA_GetState(hsai->hdmarx);
  if (status != HAL_OK) {
    s_result.diag_stage = 4u;
    return false;
  }

  s_result.diag_stage = 5u;
  return true;
}

static bool mic_pdm_start_hw(void)
{
  HAL_StatusTypeDef status = HAL_SAI_Receive_DMA(&hsai_BlockA1,
                                                 (uint8_t *)s_raw_buffer,
                                                 MIC_PDM_RAW_BUFFER_WORDS);

  s_result.rx_start_status = (uint8_t)status;
  s_result.sai_state = (uint8_t)HAL_SAI_GetState(&hsai_BlockA1);
  s_result.sai_error = HAL_SAI_GetError(&hsai_BlockA1);
  s_result.sai_clk_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI1);
  s_result.dma_state = (hsai_BlockA1.hdmarx != NULL) ? (uint8_t)HAL_DMA_GetState(hsai_BlockA1.hdmarx) : 0u;
  s_result.dma_cndtr = ((hsai_BlockA1.hdmarx != NULL) && (hsai_BlockA1.hdmarx->Instance != NULL))
                         ? hsai_BlockA1.hdmarx->Instance->CNDTR
                         : 0u;

  if (status != HAL_OK) {
    s_result.diag_stage = 6u;
    return false;
  }

  s_result.diag_stage = 7u;
  return true;
}

static void mic_pdm_stop_hw(void)
{
  (void)HAL_SAI_DMAStop(&hsai_BlockA1);
  /* Full HAL_SAI_DeInit() is deferred to mic_pdm_prepare_sai() so that
   * every capture cycle gets exactly one balanced MspDeInit -> MspInit pair.
   */
}

static void mic_pdm_reset_result(void)
{
  memset(&s_result, 0, sizeof(s_result));
  s_result.diag_stage = 0u;
  s_result.valid = false;
  s_result.last_sample = 0;
  s_result.raw_popcount_min = 32u;
  s_result.raw_popcount_max = 0u;
  memset(s_cic_int, 0, sizeof(s_cic_int));
  memset(s_cic_comb, 0, sizeof(s_cic_comb));
  s_cic_count = 0u;
  s_cic_settle = MIC_PDM_CIC_SETTLE_SAMPLES;

  /* Build byte-at-a-time CIC4 lookup tables (once). */
  if (!s_cic_luts_ready) {
    uint32_t b;
    for (b = 0u; b < 256u; b++) {
      int32_t i0 = 0, i1 = 0, i2 = 0, i3 = 0;
      int bit;
      for (bit = 7; bit >= 0; bit--) {
        int32_t x = ((b >> bit) & 1u) ? 1 : -1;
        i0 += x;
        i1 += i0;
        i2 += i1;
        i3 += i2;
      }
      s_cic_lut0[b] = (int16_t)i0;
      s_cic_lut1[b] = (int16_t)i1;
      s_cic_lut2[b] = (int16_t)i2;
      s_cic_lut3[b] = (int16_t)i3;
    }
    s_cic_luts_ready = true;
  }

  s_gtz_n = 0u;
  s_dc_est = 0.0f;
#if GOERTZEL_SCAN_MODE
  memset(s_gtz_scan_s1, 0, sizeof(s_gtz_scan_s1));
  memset(s_gtz_scan_s2, 0, sizeof(s_gtz_scan_s2));
  memset(s_gtz_scan_peak, 0, sizeof(s_gtz_scan_peak));
  if (!s_gtz_scan_coeff_ready) {
    uint32_t i;
    for (i = 0u; i < GOERTZEL_SCAN_BINS; i++) {
      uint32_t kk = GOERTZEL_SCAN_K_MIN + i * GOERTZEL_SCAN_K_STEP;
      s_gtz_scan_coeff[i] = 2.0f * cosf(2.0f * 3.14159265f * (float)kk / (float)GOERTZEL_N);
    }
    s_gtz_scan_coeff_ready = true;
  }
#elif SPIKE_DETECT_MODE
  s_spike_win_energy = 0.0f;
  s_spike_win_n = 0u;
  s_spike_peak = 0.0f;
  s_spike_count_3x = 0u;
  s_spike_count_5x = 0u;
  s_spike_peak_count = 0u;
  s_spike_post_peak = false;
  s_spike_fell_below = false;
  s_spike_smooth_e = 0.0f;
  s_spike_was_rising = false;
  s_spike_prev_smooth = 0.0f;
  memset(s_spike_env_trace, 0, sizeof(s_spike_env_trace));
  s_spike_env_count = 0u;
  s_spike_env_ready = false;
  /* bg and bg_valid are preserved across captures (set by calibration) */
#else
  s_gtz_s1 = 0.0f;
  s_gtz_s2 = 0.0f;
#endif

#if ACCUM_FFT_ENABLE
  memset(s_afft_s1, 0, sizeof(s_afft_s1));
  memset(s_afft_s2, 0, sizeof(s_afft_s2));
  memset(s_afft_peak, 0, sizeof(s_afft_peak));
  s_afft_n = 0u;
  s_afft_blocks = 0u;
  s_fft_ready = false;
  if (!s_afft_coeff_ready) {
    uint32_t i;
    for (i = 0u; i < ACCUM_FFT_BINS; i++) {
      s_afft_coeff[i] = 2.0f * cosf(2.0f * 3.14159265f * (float)s_afft_k[i] / (float)ACCUM_FFT_N);
    }
    s_afft_coeff_ready = true;
  }
#endif
}

/* Track whether the raw DMA stream looks alive before any pseudo-PCM processing happens. */
static void mic_pdm_update_transport_metrics(uint32_t start_index)
{
  uint32_t word_offset;

  for (word_offset = 0u; word_offset < MIC_PDM_HALF_BUFFER_WORDS; word_offset++) {
    uint32_t raw_word = s_raw_buffer[start_index + word_offset];
    uint8_t ones = (uint8_t)__builtin_popcount((unsigned int)raw_word);

    if ((s_result.pdm_words == 0u) && (word_offset == 0u)) {
      s_result.raw_first_word = raw_word;
    }

    s_result.raw_last_word = raw_word;
    if (raw_word == 0u) {
      s_result.raw_zero_words += 1u;
    }
    if (raw_word == 0xFFFFFFFFu) {
      s_result.raw_full_words += 1u;
    }
    if (ones < s_result.raw_popcount_min) {
      s_result.raw_popcount_min = ones;
    }
    if (ones > s_result.raw_popcount_max) {
      s_result.raw_popcount_max = ones;
    }
  }
}

static void mic_pdm_process_half(uint32_t start_index)
{
  uint32_t word_offset;

  mic_pdm_update_transport_metrics(start_index);

  for (word_offset = 0u; word_offset < MIC_PDM_HALF_BUFFER_WORDS; word_offset++) {
    uint32_t raw_word = s_raw_buffer[start_index + word_offset];
    uint8_t pdm_byte;

    /* Skip mic wakeup transients (IM67D120, ~25 ms). */
    if (s_wakeup_skip > 0u) {
      s_wakeup_skip--;
      continue;
    }

    /* With SAI_SLOTACTIVE_2 only, each 32-bit DMA word holds one 8-bit
     * slot value (pair2-L from D2) at bits [7:0].  Upper 24 bits are padding.
     */
    pdm_byte = (uint8_t)(raw_word & 0xFFu);

    /* Byte-at-a-time CIC4 integrator update.
     * Instead of iterating 8 bits individually, advance all 4 integrator
     * stages in one step using precomputed lookup tables.  The tables
     * encode the cumulative contribution of the 8 bits to each stage.
     *
     * Mathematical identity (for stage k, processing 8 bits b7..b0):
     *   new_int[0] = old_int[0] + lut0[byte]
     *   new_int[1] = old_int[1] + 8*old_int[0] + lut1[byte]
     *   new_int[2] = old_int[2] + 8*old_int[1] + 36*old_int[0] + lut2[byte]
     *   new_int[3] = old_int[3] + 8*old_int[2] + 36*old_int[1] + 120*old_int[0] + lut3[byte]
     *
     * Coefficients: C(8,1)=8, C(9,2)=36, C(10,3)=120 — triangular sums
     * from running the integrator cascade on 8 sequential inputs.
     */
    {
      uint32_t ci0 = s_cic_int[0];
      uint32_t ci1 = s_cic_int[1];
      uint32_t ci2 = s_cic_int[2];
      s_cic_int[3] += 8u * ci2 + 36u * ci1 + 120u * ci0 + (uint32_t)(int32_t)s_cic_lut3[pdm_byte];
      s_cic_int[2] += 8u * ci1 + 36u * ci0 + (uint32_t)(int32_t)s_cic_lut2[pdm_byte];
      s_cic_int[1] += 8u * ci0 + (uint32_t)(int32_t)s_cic_lut1[pdm_byte];
      s_cic_int[0] += (uint32_t)(int32_t)s_cic_lut0[pdm_byte];
    }

    s_cic_count++;
    /* R=32 bits = 4 bytes per decimated output. */
    if (s_cic_count >= (MIC_PDM_CIC_DECIMATION / 8u)) {
      uint32_t y = s_cic_int[MIC_PDM_CIC_ORDER - 1u];
      int32_t pcm32;
      int16_t pcm;
      uint16_t abs_pcm;
      uint32_t k;

      s_cic_count = 0u;

      /* Comb cascade (runs at decimated output rate). */
      for (k = 0u; k < MIC_PDM_CIC_ORDER; k++) {
        uint32_t tmp = y - s_cic_comb[k];
        s_cic_comb[k] = y;
        y = tmp;
      }

        /* CIC4 gain = R^N = 32^4 = 1048576 (21 bits); >> 9 keeps noise floor
         * manageable while fitting int16 range.
         */
        pcm32 = (int32_t)y >> 9;

        /* Discard the first CIC output samples: comb delay registers start
         * at zero so the initial outputs carry a large startup transient
         * that would corrupt peak/avg statistics.
         */
        if (s_cic_settle > 0u) {
          s_cic_settle--;
          continue;
        }

        pcm = (int16_t)(pcm32 - s_bg_offset);
        s_result.last_sample = pcm;
        s_result.pcm_samples++;

        /* DC-removal: first-order IIR high-pass, ~37 Hz cutoff at 46.875 kHz.
         * Removes DC leak that would otherwise dominate low-k Goertzel bins.
         */
        {
          float xf = (float)pcm;
          s_dc_est += (1.0f / 512.0f) * (xf - s_dc_est);  /* α ≈ 0.002 → cutoff ~14.6 Hz */
          float ac = xf - s_dc_est;

#if GOERTZEL_SCAN_MODE
        /* Goertzel frequency scan: k=1..31 in parallel */
        {
          uint32_t gi;
          for (gi = 0u; gi < GOERTZEL_SCAN_BINS; gi++) {
            float s0 = s_gtz_scan_coeff[gi] * s_gtz_scan_s1[gi]
                      - s_gtz_scan_s2[gi] + ac;
            s_gtz_scan_s2[gi] = s_gtz_scan_s1[gi];
            s_gtz_scan_s1[gi] = s0;
          }
          s_gtz_n++;
          if (s_gtz_n >= GOERTZEL_N) {
            for (gi = 0u; gi < GOERTZEL_SCAN_BINS; gi++) {
              float c = s_gtz_scan_coeff[gi];
              float gs1 = s_gtz_scan_s1[gi];
              float gs2 = s_gtz_scan_s2[gi];
              float mag_sq = gs1 * gs1 + gs2 * gs2 - c * gs1 * gs2;
              float mag = (mag_sq > 0.0f) ? sqrtf(mag_sq) : 0.0f;
              if (s_gtz_calibrating) {
                s_gtz_scan_peak[gi] += mag;  /* sum for averaging */
              } else {
                if (mag > s_gtz_scan_peak[gi]) {
                  s_gtz_scan_peak[gi] = mag;  /* track max */
                }
              }
            }
            if (s_gtz_calibrating) {
              s_gtz_cal_count++;
            }
            s_result.goertzel_bins_total++;
            memset(s_gtz_scan_s1, 0, sizeof(s_gtz_scan_s1));
            memset(s_gtz_scan_s2, 0, sizeof(s_gtz_scan_s2));
            s_gtz_n = 0u;
          }
        }
#elif SPIKE_DETECT_MODE
        /* Broadband spike detector: energy per window */
        {
          s_spike_win_energy += ac * ac;
          s_spike_win_n++;
          if (s_spike_win_n >= SPIKE_WIN_SIZE) {
            float win_e = s_spike_win_energy;
            s_result.goertzel_bins_total++;  /* count windows */
            if (s_gtz_calibrating) {
              /* Calibration: update background estimate (simple running average) */
              s_gtz_cal_count++;
              s_spike_bg += (win_e - s_spike_bg) / (float)s_gtz_cal_count;
            } else {
              /* Measurement: track peak and count spikes relative to bg */
              if (win_e > s_spike_peak) {
                s_spike_peak = win_e;
              }
              if (s_spike_bg_valid && s_spike_bg > 0.0f) {
                /* EMA smooth energy for peak detection (α=0.25, cutoff ~7 Hz) */
                s_spike_smooth_e += 0.25f * (win_e - s_spike_smooth_e);
                if (s_spike_env_count < MIC_PDM_ENVELOPE_TRACE_BINS) {
                  float env10 = 10.0f * s_spike_smooth_e / s_spike_bg;
                  s_spike_env_trace[s_spike_env_count++] =
                      (env10 > 65535.0f) ? 65535u : (uint16_t)(env10 + 0.5f);
                }
                /* Hysteresis + direction guard: after a counted peak, EMA must
                 * fall below REARM (1.5×bg), then start rising, to re-arm.
                 * Detection stays at DETECT (3×bg). Band 1.5×–3× = hysteresis. */
                if (s_spike_post_peak) {
                  if (s_spike_smooth_e < SPIKE_REARM_FACTOR * s_spike_bg) {
                    s_spike_fell_below = true;
                  }
                  if (s_spike_fell_below
                      && s_spike_smooth_e > s_spike_prev_smooth) {
                    /* EMA fell below 1.5×bg and is now rising → new pulse */
                    s_spike_post_peak = false;
                    s_spike_fell_below = false;
                    s_spike_was_rising = false;
                  }
                }
                if (!s_spike_post_peak) {
                  bool rising = (s_spike_smooth_e > s_spike_prev_smooth)
                                && (s_spike_smooth_e > SPIKE_DETECT_FACTOR * s_spike_bg);
                  if (!rising && s_spike_was_rising) {
                    /* EMA peaked above 3×bg and started falling — one click */
                    s_spike_peak_count++;
                    s_spike_post_peak = true;
                    s_spike_fell_below = false;
                    s_spike_was_rising = false;
                  } else {
                    s_spike_was_rising = rising;
                  }
                }
                if (win_e > SPIKE_DETECT_FACTOR * s_spike_bg) { s_spike_count_3x++; }
                if (win_e > 5.0f * s_spike_bg) { s_spike_count_5x++; }
                s_spike_prev_smooth = s_spike_smooth_e;
              }
            }
            s_spike_win_energy = 0.0f;
            s_spike_win_n = 0u;
          }
        }
#else
        /* Goertzel single-frequency detector */
        {
          float s0 = GOERTZEL_COEFF * s_gtz_s1 - s_gtz_s2 + ac;
          s_gtz_s2 = s_gtz_s1;
          s_gtz_s1 = s0;
          s_gtz_n++;
          if (s_gtz_n >= GOERTZEL_N) {
            float mag_sq = s_gtz_s1 * s_gtz_s1 + s_gtz_s2 * s_gtz_s2
                         - GOERTZEL_COEFF * s_gtz_s1 * s_gtz_s2;
            float mag = (mag_sq > 0.0f) ? sqrtf(mag_sq) : 0.0f;
            s_result.goertzel_bins_total++;
            if (s_gtz_calibrating) {
              s_gtz_cal_count++;
              float delta = mag - s_gtz_cal_mean;
              s_gtz_cal_mean += delta / (float)s_gtz_cal_count;
              float delta2 = mag - s_gtz_cal_mean;
              s_gtz_cal_m2 += delta * delta2;
            } else if ((s_gtz_threshold > 0.0f) && (mag > s_gtz_threshold)) {
              s_result.goertzel_bins_active++;
            }
            s_gtz_s1 = 0.0f;
            s_gtz_s2 = 0.0f;
            s_gtz_n = 0u;
          }
        }
#endif

#if ACCUM_FFT_ENABLE
        /* Accumulated Goertzel: update 32 bins per sample, evaluate every N=4096. */
        {
          uint32_t gi;
          for (gi = 0u; gi < ACCUM_FFT_BINS; gi++) {
            float s0 = s_afft_coeff[gi] * s_afft_s1[gi] - s_afft_s2[gi] + ac;
            s_afft_s2[gi] = s_afft_s1[gi];
            s_afft_s1[gi] = s0;
          }
          s_afft_n++;
          if (s_afft_n >= ACCUM_FFT_N) {
            for (gi = 0u; gi < ACCUM_FFT_BINS; gi++) {
              float c = s_afft_coeff[gi];
              float gs1 = s_afft_s1[gi];
              float gs2 = s_afft_s2[gi];
              float mag_sq = gs1 * gs1 + gs2 * gs2 - c * gs1 * gs2;
              float mag = (mag_sq > 0.0f) ? sqrtf(mag_sq) : 0.0f;
              if (mag > s_afft_peak[gi]) {
                s_afft_peak[gi] = mag;
              }
            }
            memset(s_afft_s1, 0, sizeof(s_afft_s1));
            memset(s_afft_s2, 0, sizeof(s_afft_s2));
            s_afft_n = 0u;
            s_afft_blocks++;
          }
        }
#endif
        }  /* end DC-removal scope */

        abs_pcm = (uint16_t)((pcm < 0) ? -pcm : pcm);
        s_total_abs_sum += (uint64_t)abs_pcm;
        s_total_signed_sum += (int64_t)pcm32;
        if (abs_pcm > s_result.peak_abs) {
          s_result.peak_abs = abs_pcm;
        }
    }
  }

  s_result.block_count++;
  s_result.pdm_words += MIC_PDM_HALF_BUFFER_WORDS;
}

/* Finish the capture in thread context so callbacks stay short and deterministic. */
static void mic_pdm_finalize(void)
{
  uint32_t sai_error = HAL_SAI_GetError(&hsai_BlockA1);
  DMA_HandleTypeDef *hdmarx = hsai_BlockA1.hdmarx;

  s_result.sai_enabled = ((hsai_BlockA1.Instance->CR1 & SAI_xCR1_SAIEN) != 0u);
  s_result.dma_enabled = ((hsai_BlockA1.Instance->CR1 & SAI_xCR1_DMAEN) != 0u);
  s_result.ckstr = (hsai_BlockA1.Init.ClockStrobing == SAI_CLOCKSTROBING_RISINGEDGE);
  s_result.pdm_enabled = ((SAI1->PDMCR & SAI_PDMCR_PDMEN) != 0u);
  s_result.sai_state = (uint8_t)HAL_SAI_GetState(&hsai_BlockA1);
  s_result.dma_state = ((hdmarx != NULL) ? (uint8_t)HAL_DMA_GetState(hdmarx) : 0u);
  s_result.sai_clk_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI1);
  s_result.sai_error = sai_error;
  s_result.dma_cndtr = ((hdmarx != NULL) && (hdmarx->Instance != NULL)) ? hdmarx->Instance->CNDTR : 0u;
  s_result.sai_sr = hsai_BlockA1.Instance->SR;
  s_result.capture_ms = HAL_GetTick() - s_start_tick_ms;
  s_result.overflow = (s_overflow != 0u) || ((sai_error & HAL_SAI_ERROR_OVR) != 0u);
  s_result.dma_error = (s_dma_error != 0u) || ((sai_error & HAL_SAI_ERROR_DMA) != 0u);
  s_result.autodetect_failed = false;
  s_result.valid = (s_result.pcm_samples != 0u) && !s_result.dma_error;

  if (s_result.pcm_samples > 0u) {
    s_result.avg_abs = (uint16_t)(s_total_abs_sum / s_result.pcm_samples);
    s_result.avg_signed = (int32_t)(s_total_signed_sum / (int64_t)s_result.pcm_samples);
  }

#if GOERTZEL_SCAN_MODE
  {
    uint8_t peak_k = GOERTZEL_SCAN_K_MIN;
    float peak_mag = 0.0f;
    uint32_t gi;
    for (gi = 0u; gi < GOERTZEL_SCAN_BINS; gi++) {
      float diff = s_gtz_scan_peak[gi] - s_gtz_scan_bg[gi];
      if (diff > peak_mag) {
        peak_mag = diff;
        peak_k = (uint8_t)(GOERTZEL_SCAN_K_MIN + gi * GOERTZEL_SCAN_K_STEP);
      }
    }
    s_result.scan_peak_k = peak_k;
    s_result.scan_peak_mag = (uint32_t)(peak_mag > 0.0f ? peak_mag + 0.5f : 0.0f);
    {
      uint32_t target_idx = (22u - GOERTZEL_SCAN_K_MIN) / GOERTZEL_SCAN_K_STEP;
      float target_diff = s_gtz_scan_peak[target_idx]
                        - s_gtz_scan_bg[target_idx];
      s_result.scan_target_mag = (uint32_t)(target_diff > 0.0f ? target_diff + 0.5f : 0.0f);
    }
  }
#elif SPIKE_DETECT_MODE
  {
    /* Repurpose existing result fields for spike data:
     * scan_peak_mag  = peak window energy (sqrt for readability)
     * scan_target_mag = background energy (sqrt)
     * scan_peak_k    = peak/bg ratio (capped at 255)
     * goertzel_bins_active = spike count (3× threshold)
     */
    float peak_rms = (s_spike_peak > 0.0f) ? sqrtf(s_spike_peak / (float)SPIKE_WIN_SIZE) : 0.0f;
    float bg_rms = (s_spike_bg > 0.0f) ? sqrtf(s_spike_bg / (float)SPIKE_WIN_SIZE) : 0.0f;
    float ratio = (bg_rms > 0.0f) ? (peak_rms / bg_rms) : 0.0f;
    s_result.scan_peak_mag = (uint32_t)(peak_rms + 0.5f);
    s_result.scan_target_mag = (uint32_t)(bg_rms + 0.5f);
    s_result.scan_peak_k = (ratio > 255.0f) ? 255u : (uint8_t)(ratio + 0.5f);
    s_result.goertzel_bins_active = s_spike_count_3x;
    s_result.spike_clusters = s_spike_peak_count;
    s_spike_env_ready = (s_spike_env_count > 0u);
  }
#endif

  if (!s_result.valid && (s_result.diag_stage < 8u)) {
    s_result.diag_stage = 8u;
  }

#if ACCUM_FFT_ENABLE
  /* Convert accumulated peak magnitudes to normalised uint16 for logging.
   * Normalise by N/2 so a pure tone of amplitude A gives ~A.
   */
  if (s_result.valid && s_afft_blocks > 0u) {
    uint32_t bi;
    for (bi = 0u; bi < ACCUM_FFT_BINS; bi++) {
      float norm = s_afft_peak[bi] * 2.0f / (float)ACCUM_FFT_N;
      s_fft_mags[bi] = (norm > 65535.0f) ? 65535u : (uint16_t)(norm + 0.5f);
    }
    s_fft_ready = true;
  }
#endif

  mic_pdm_stop_hw();
  mic_pdm_restore_shared_pins();

  s_state = MIC_PDM_STATE_IDLE;
  s_result_ready = true;
}

/* Validate the CubeMX-created handles once and leave the mic block in idle-safe state. */
bool mic_pdm_init(void)
{
  if (s_initialized) {
    return true;
  }

  if ((hsai_BlockA1.Instance != SAI1_Block_A) || (hdma_sai1_a.Instance == NULL)) {
    return false;
  }

  mic_pdm_restore_shared_pins();
  mic_pdm_reset_result();
  s_pending_halves = 0u;
  s_overflow = 0u;
  s_dma_error = 0u;
  s_state = MIC_PDM_STATE_IDLE;
  s_result_ready = false;
  s_initialized = true;
  return true;
}

/* Own power, pins, SAI and DMA only for the brief microphone acquisition window. */
bool mic_pdm_start(void)
{
  if (!s_initialized || (s_state != MIC_PDM_STATE_IDLE)) {
    return false;
  }

  mic_pdm_reset_result();
  memset(s_raw_buffer, 0, sizeof(s_raw_buffer));
  s_pending_halves = 0u;
  s_overflow = 0u;
  s_dma_error = 0u;
  s_result_ready = false;
  s_start_tick_ms = HAL_GetTick();
  s_stop_after_ms = 0u;
  s_total_abs_sum = 0u;
  s_total_signed_sum = 0;
  s_current_ckstr = false;
  s_wakeup_skip = MIC_PDM_WAKEUP_SKIP_WORDS;

  mic_pdm_power_on();
  if (!mic_pdm_prepare_sai(s_current_ckstr)) {
    mic_pdm_restore_shared_pins();
    return false;
  }

  mic_pdm_configure_shared_pins_for_capture();
  if (!mic_pdm_start_hw()) {
    mic_pdm_stop_hw();
    mic_pdm_restore_shared_pins();
    return false;
  }

  s_state = MIC_PDM_STATE_RUNNING;
  return true;
}

void mic_pdm_request_stop(void)
{
  if (s_state == MIC_PDM_STATE_RUNNING) {
    s_stop_after_ms = HAL_GetTick() + MIC_PDM_STOP_TAIL_MS;
    s_state = MIC_PDM_STATE_STOPPING;
  }
}

/* Immediately finalize capture regardless of state; result is available via take_result(). */
void mic_pdm_force_stop(void)
{
  if (s_state != MIC_PDM_STATE_IDLE) {
    mic_pdm_finalize();
  }
}

/* Drain completed DMA halves, update diagnostics and stop after the requested tail time. */
void mic_pdm_task(void)
{
  uint8_t pending;

  if (s_state == MIC_PDM_STATE_IDLE) {
    return;
  }

  pending = s_pending_halves;
  if ((pending & 0x01u) != 0u) {
    s_pending_halves &= (uint8_t)~0x01u;
    mic_pdm_process_half(0u);
  }

  if ((pending & 0x02u) != 0u) {
    s_pending_halves &= (uint8_t)~0x02u;
    mic_pdm_process_half(MIC_PDM_HALF_BUFFER_WORDS);
  }

  if (s_dma_error != 0u) {
    mic_pdm_finalize();
    return;
  }

  /* Hard timeout: if capture exceeds the safety limit, force-stop and finalize
   * so mic state can never permanently block the main loop.
   */
  if ((int32_t)(HAL_GetTick() - s_start_tick_ms) >= (int32_t)MIC_PDM_MAX_CAPTURE_MS) {
    mic_pdm_finalize();
    return;
  }

  if ((s_state == MIC_PDM_STATE_STOPPING) &&
      mic_pdm_tick_expired(s_stop_after_ms)) {
    mic_pdm_finalize();
  }
}

bool mic_pdm_is_active(void)
{
  return (s_state != MIC_PDM_STATE_IDLE);
}

bool mic_pdm_take_result(mic_pdm_result_t *result)
{
  if (!s_result_ready || (result == NULL)) {
    return false;
  }

  *result = s_result;
  s_result_ready = false;
  return true;
}

void mic_pdm_set_bg_offset(int32_t offset)
{
  s_bg_offset = offset;
}

int32_t mic_pdm_get_bg_offset(void)
{
  return s_bg_offset;
}

void mic_pdm_start_goertzel_cal(void)
{
  s_gtz_calibrating = true;
  s_gtz_cal_mean = 0.0f;
  s_gtz_cal_m2 = 0.0f;
  s_gtz_cal_count = 0u;
}

float mic_pdm_finish_goertzel_cal(void)
{
  float threshold = 0.0f;
  s_gtz_calibrating = false;
#if GOERTZEL_SCAN_MODE
  if (s_gtz_cal_count > 0u) {
    uint32_t gi;
    for (gi = 0u; gi < GOERTZEL_SCAN_BINS; gi++) {
      s_gtz_scan_bg[gi] = s_gtz_scan_peak[gi] / (float)s_gtz_cal_count;
    }
  }
#elif SPIKE_DETECT_MODE
  /* s_spike_bg was computed as running average during cal — just mark valid */
  if (s_gtz_cal_count > 0u) {
    s_spike_bg_valid = true;
  }
#else
  if (s_gtz_cal_count > 1u) {
    float variance = s_gtz_cal_m2 / (float)s_gtz_cal_count;
    float sigma = sqrtf(variance);
    threshold = s_gtz_cal_mean + 3.0f * sigma;
  } else if (s_gtz_cal_count == 1u) {
    threshold = s_gtz_cal_mean * 2.0f;
  }
  s_gtz_threshold = threshold;
#endif
  return threshold;
}

void mic_pdm_set_goertzel_threshold(float threshold)
{
  s_gtz_threshold = threshold;
}

float mic_pdm_get_goertzel_threshold(void)
{
  return s_gtz_threshold;
}

bool mic_pdm_get_scan_averages(uint16_t *out_avg, uint16_t *out_bg, uint32_t count)
{
#if GOERTZEL_SCAN_MODE
  if (count > GOERTZEL_SCAN_BINS) count = GOERTZEL_SCAN_BINS;
  /* For CAL captures, scan_peak holds accumulated sums that need averaging.
   * For MES captures, scan_peak holds per-bin max — return as-is.
   * Detect mode by checking s_gtz_calibrating (still true until finish_cal,
   * but get_scan_averages is called after finish_cal sets it false).
   * Use goertzel_bins_total: if still accumulating sums (calibration just
   * finished), divide.  Actually simplest: caller passes the label.
   * Instead: check if cal_count > 0 AND calibrating just finished.
   * Use a flag.
   */
  for (uint32_t i = 0u; i < count; i++) {
    float val = s_gtz_scan_peak[i];
    out_avg[i] = (uint16_t)(val > 65535.0f ? 65535u : (uint16_t)(val + 0.5f));
    out_bg[i] = (uint16_t)(s_gtz_scan_bg[i] > 65535.0f ? 65535u : (uint16_t)(s_gtz_scan_bg[i] + 0.5f));
  }
  return true;
#else
  (void)out_avg; (void)out_bg; (void)count;
  return false;
#endif
}

bool mic_pdm_get_fft_spectrum(uint16_t *mags, uint32_t *count)
{
#if ACCUM_FFT_ENABLE
  if (!s_fft_ready || (mags == NULL) || (count == NULL)) {
    return false;
  }
  memcpy(mags, s_fft_mags, ACCUM_FFT_BINS * sizeof(uint16_t));
  *count = ACCUM_FFT_BINS;
  s_fft_ready = false;
  return true;
#else
  (void)mags; (void)count;
  return false;
#endif
}

bool mic_pdm_get_envelope_trace(const uint16_t **trace, uint32_t *count)
{
#if SPIKE_DETECT_MODE
  if (!s_spike_env_ready || (trace == NULL) || (count == NULL)) {
    return false;
  }
  *trace = s_spike_env_trace;
  *count = s_spike_env_count;
  s_spike_env_ready = false;
  return true;
#else
  (void)trace;
  (void)count;
  return false;
#endif
}

/* DMA IRQ path only forwards control to HAL so half/full callbacks can mark pending work. */
void mic_pdm_dma_irqhandler(void)
{
  DMA_HandleTypeDef *hdmarx = hsai_BlockA1.hdmarx;

  if (hdmarx != NULL) {
    HAL_DMA_IRQHandler(hdmarx);
  }
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai != &hsai_BlockA1) {
    return;
  }

  if ((s_pending_halves & 0x01u) != 0u) {
    s_overflow = 1u;
  }
  s_pending_halves |= 0x01u;
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai != &hsai_BlockA1) {
    return;
  }

  if ((s_pending_halves & 0x02u) != 0u) {
    s_overflow = 1u;
  }
  s_pending_halves |= 0x02u;
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai != &hsai_BlockA1) {
    return;
  }

  s_dma_error = 1u;
}
