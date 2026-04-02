#include "mic_pdm.h"

#include <string.h>

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
#define MIC_PDM_RAW_BUFFER_WORDS      2048u
/* Raw words processed per DMA half-complete callback. */
#define MIC_PDM_HALF_BUFFER_WORDS     (MIC_PDM_RAW_BUFFER_WORDS / 2u)
/* Post-stop tail time before capture is finalized. */
#define MIC_PDM_STOP_TAIL_MS          32u
/* Hard safety limit for any single capture; force-finalize if exceeded. */
#define MIC_PDM_MAX_CAPTURE_MS        5000u
/* CIC decimation filter: order 5, R=16 → 16-bit PDM→PCM conversion.
 * With 8 raw PDM bits per DMA word (pair2-L byte) at ~93.75 kHz frame rate
 * the resulting PCM sample rate is 93750 × 8 / 16 ≈ 46.875 kHz.
 */
#define MIC_PDM_CIC_ORDER             5u
#define MIC_PDM_CIC_DECIMATION        16u
/* IM67D120 needs ~25 ms after clock restart before PDM output stabilises.
 * Skip this many DMA words at the start of each capture to avoid wakeup
 * transients polluting the peak/avg metrics.
 * At ~93.75 kHz frame rate: 2400 words ≈ 25.6 ms.
 */
#define MIC_PDM_WAKEUP_SKIP_WORDS    2400u
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
static int32_t s_bg_offset = 0;       /* background noise offset from Enter calibration */
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
  hsai->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_96K;
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
    int bit;

    /* Skip mic wakeup transients (IM67D120, ~25 ms). */
    if (s_wakeup_skip > 0u) {
      s_wakeup_skip--;
      continue;
    }

    /* With SAI_SLOTACTIVE_2 only, each 32-bit DMA word holds one 8-bit
     * slot value (pair2-L from D2) at bits [7:0].  Upper 24 bits are padding.
     */
    pdm_byte = (uint8_t)(raw_word & 0xFFu);

    /* Feed 8 PDM bits through CIC4 decimator (MSB = earliest captured).
     * Integrators use uint32_t for correct modular wrap on long captures.
     */
    for (bit = 7; bit >= 0; bit--) {
      uint32_t x = ((pdm_byte >> bit) & 1u) ? 1u : 0xFFFFFFFFu;
      uint32_t k;

      /* Integrator cascade — must cover all CIC_ORDER stages. */
      s_cic_int[0] += x;
      for (k = 1u; k < MIC_PDM_CIC_ORDER; k++) {
        s_cic_int[k] += s_cic_int[k - 1u];
      }

      s_cic_count++;
      if (s_cic_count >= MIC_PDM_CIC_DECIMATION) {
        uint32_t y = s_cic_int[MIC_PDM_CIC_ORDER - 1u];
        int32_t pcm32;
        int16_t pcm;
        uint16_t abs_pcm;

        s_cic_count = 0u;

        /* Comb cascade (runs at decimated output rate). */
        for (k = 0u; k < MIC_PDM_CIC_ORDER; k++) {
          uint32_t tmp = y - s_cic_comb[k];
          s_cic_comb[k] = y;
          y = tmp;
        }

        /* CIC5 gain = R^N = 16^5 = 1048576 (21 bits); >> 5 for signed 16-bit. */
        pcm32 = (int32_t)y >> 5;
        pcm = (int16_t)(pcm32 - s_bg_offset);
        s_result.last_sample = pcm;
        s_result.pcm_samples++;

        abs_pcm = (uint16_t)((pcm < 0) ? -pcm : pcm);
        s_total_abs_sum += (uint64_t)abs_pcm;
        s_total_signed_sum += (int64_t)pcm32;
        if (abs_pcm > s_result.peak_abs) {
          s_result.peak_abs = abs_pcm;
        }
        s_result.avg_abs = (s_result.pcm_samples > 0u)
                           ? (uint16_t)(s_total_abs_sum / s_result.pcm_samples)
                           : 0u;
        s_result.avg_signed = (s_result.pcm_samples > 0u)
                              ? (int32_t)(s_total_signed_sum / (int64_t)s_result.pcm_samples)
                              : 0;
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
  if (!s_result.valid && (s_result.diag_stage < 8u)) {
    s_result.diag_stage = 8u;
  }

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
