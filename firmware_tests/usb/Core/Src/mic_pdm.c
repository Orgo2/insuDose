#include "mic_pdm.h"

#include <string.h>

#include "main.h"
#include "stm32wbxx_hal_dma.h"
#include "stm32wbxx_hal_gpio_ex.h"
#include "stm32wbxx_hal_rcc_ex.h"

#define MIC_PDM_DMA_CHANNEL              DMA1_Channel1
#define MIC_PDM_DMA_IRQn                 DMA1_Channel1_IRQn
#define MIC_PDM_DMA_REQUEST              DMA_REQUEST_SAI1_A
#define MIC_PDM_RAW_BUFFER_WORDS         2048u
#define MIC_PDM_HALF_BUFFER_WORDS        (MIC_PDM_RAW_BUFFER_WORDS / 2u)
#define MIC_PDM_DECIMATION_BITS          64u
#define MIC_PDM_PCM_SAMPLES_PER_HALF     (MIC_PDM_HALF_BUFFER_WORDS / 4u)
#define MIC_PDM_STOP_TAIL_MS             32u
#define MIC_PDM_AUTODETECT_PROBE_MS      6u
#define MIC_PDM_FRAME_LENGTH_BITS        16u
#define MIC_PDM_ACTIVE_FRAME_BITS        1u
#define MIC_PDM_GOERTZEL_COEFF_Q14       (-12540)
#define MIC_PDM_DEBUG_ALWAYS_POWERED     1u

typedef enum
{
  MIC_PDM_STATE_IDLE = 0,
  MIC_PDM_STATE_RUNNING,
  MIC_PDM_STATE_STOPPING
} mic_pdm_state_t;

static DMA_HandleTypeDef s_mic_dma;
static uint16_t s_raw_buffer[MIC_PDM_RAW_BUFFER_WORDS];
static volatile uint8_t s_pending_halves = 0u;
static volatile uint8_t s_overflow = 0u;
static volatile uint8_t s_dma_error = 0u;
static uint32_t s_start_tick_ms = 0u;
static uint32_t s_stop_after_ms = 0u;
static uint64_t s_total_abs_sum = 0u;
static mic_pdm_state_t s_state = MIC_PDM_STATE_IDLE;
static bool s_initialized = false;
static bool s_result_ready = false;
static bool s_preferred_ckstr = false;
static bool s_current_ckstr = false;
static bool s_autodetect_retry_done = false;
static uint32_t s_autodetect_probe_ms = 0u;
static mic_pdm_result_t s_result;

static bool mic_pdm_tick_expired(uint32_t deadline_ms);
static void mic_pdm_configure_shared_pins_for_capture(void);
static void mic_pdm_restore_shared_pins(void);
static void mic_pdm_power_on(void);
static void mic_pdm_power_off(void);
static bool mic_pdm_configure_clock(void);
static bool mic_pdm_configure_dma(void);
static void mic_pdm_configure_sai(bool ckstr);
static bool mic_pdm_start_hw(void);
static void mic_pdm_stop_hw(void);
static void mic_pdm_reset_result(void);
static void mic_pdm_process_half(uint32_t start_index);
static bool mic_pdm_has_progress(void);
static bool mic_pdm_restart_with_polarity(bool ckstr);
static void mic_pdm_finalize(void);
static void mic_pdm_dma_half_complete(DMA_HandleTypeDef *hdma);
static void mic_pdm_dma_complete(DMA_HandleTypeDef *hdma);
static void mic_pdm_dma_error(DMA_HandleTypeDef *hdma);

static bool mic_pdm_tick_expired(uint32_t deadline_ms)
{
  return ((int32_t)(HAL_GetTick() - deadline_ms) >= 0);
}

static void mic_pdm_configure_shared_pins_for_capture(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
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

static void mic_pdm_restore_shared_pins(void)
{
  GPIO_InitTypeDef gpio = {0};

  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9);

  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  mic_pdm_power_off();
}

static bool mic_pdm_configure_clock(void)
{
  RCC_PeriphCLKInitTypeDef periph_clk = {0};

  periph_clk.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  periph_clk.PLLSAI1.PLLN = 24;
  periph_clk.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  periph_clk.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  periph_clk.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;

  return (HAL_RCCEx_PeriphCLKConfig(&periph_clk) == HAL_OK);
}

static bool mic_pdm_configure_dma(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  s_mic_dma.Instance = MIC_PDM_DMA_CHANNEL;
  s_mic_dma.Init.Request = MIC_PDM_DMA_REQUEST;
  s_mic_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
  s_mic_dma.Init.PeriphInc = DMA_PINC_DISABLE;
  s_mic_dma.Init.MemInc = DMA_MINC_ENABLE;
  s_mic_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  s_mic_dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  s_mic_dma.Init.Mode = DMA_CIRCULAR;
  s_mic_dma.Init.Priority = DMA_PRIORITY_HIGH;

  if (HAL_DMA_Init(&s_mic_dma) != HAL_OK) {
    return false;
  }

  s_mic_dma.Parent = NULL;
  s_mic_dma.XferHalfCpltCallback = mic_pdm_dma_half_complete;
  s_mic_dma.XferCpltCallback = mic_pdm_dma_complete;
  s_mic_dma.XferErrorCallback = mic_pdm_dma_error;
  s_mic_dma.XferAbortCallback = NULL;

  HAL_NVIC_SetPriority(MIC_PDM_DMA_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(MIC_PDM_DMA_IRQn);
  return true;
}

static void mic_pdm_configure_sai(bool ckstr)
{
  uint32_t mckdiv;
  uint32_t sai_clk_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI1);
  uint32_t tmpval = (sai_clk_hz * 10u) / (96000u * 256u);

  mckdiv = tmpval / 10u;
  if ((tmpval % 10u) > 8u) {
    mckdiv += 1u;
  }

  __HAL_RCC_SAI1_CLK_ENABLE();
  __HAL_RCC_SAI1_FORCE_RESET();
  __HAL_RCC_SAI1_RELEASE_RESET();

  SAI1_Block_A->CR1 = 0u;
  SAI1_Block_A->CR2 = 0u;
  SAI1_Block_A->FRCR = 0u;
  SAI1_Block_A->SLOTR = 0u;
  SAI1_Block_A->IMR = 0u;
  SAI1_Block_A->CLRFR = 0xFFFFFFFFu;
  SAI1_Block_A->CR2 |= SAI_xCR2_FFLUSH;

  SAI1_Block_A->CR1 = SAI_xCR1_MODE_0 |
                      SAI_xCR1_DS_2 |
                      SAI_xCR1_MONO |
                      (ckstr ? SAI_xCR1_CKSTR : 0u) |
                      (mckdiv << 20);

  /* PDM capture with one active data line uses a 16-bit frame and one slot. */
  SAI1_Block_A->FRCR = ((MIC_PDM_FRAME_LENGTH_BITS - 1u) << SAI_xFRCR_FRL_Pos) |
                       ((MIC_PDM_ACTIVE_FRAME_BITS - 1u) << SAI_xFRCR_FSALL_Pos);
  SAI1_Block_A->SLOTR = 0u;

  /* Pair 2 only, one active microphone on D2/CK2. */
  SAI1->PDMDLY = 0u;
  SAI1->PDMCR = 0u |
                SAI_PDMCR_CKEN2 |
                SAI_PDMCR_PDMEN;
}

static bool mic_pdm_start_hw(void)
{
  SAI1_Block_A->CLRFR = 0xFFFFFFFFu;
  SAI1_Block_A->CR2 |= SAI_xCR2_FFLUSH;

  if (HAL_DMA_Start_IT(&s_mic_dma,
                       (uint32_t)&SAI1_Block_A->DR,
                       (uint32_t)s_raw_buffer,
                       MIC_PDM_RAW_BUFFER_WORDS) != HAL_OK) {
    return false;
  }

  SAI1_Block_A->CR1 |= SAI_xCR1_DMAEN;
  SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
  return true;
}

static void mic_pdm_stop_hw(void)
{
  SAI1_Block_A->CR1 &= ~SAI_xCR1_DMAEN;
  SAI1_Block_A->CR1 &= ~SAI_xCR1_SAIEN;
  SAI1_Block_A->CLRFR = 0xFFFFFFFFu;
  (void)HAL_DMA_Abort(&s_mic_dma);
  SAI1->PDMCR &= ~SAI_PDMCR_PDMEN;
  __HAL_RCC_SAI1_CLK_DISABLE();
}

static void mic_pdm_reset_result(void)
{
  memset(&s_result, 0, sizeof(s_result));
  s_result.valid = false;
  s_result.last_sample = 0;
}

static bool mic_pdm_has_progress(void)
{
  return (s_result.pcm_samples != 0u) ||
         (s_result.pdm_words != 0u) ||
         (s_pending_halves != 0u) ||
         (s_mic_dma.Instance->CNDTR != MIC_PDM_RAW_BUFFER_WORDS);
}

static bool mic_pdm_restart_with_polarity(bool ckstr)
{
  mic_pdm_stop_hw();
  memset(s_raw_buffer, 0, sizeof(s_raw_buffer));
  s_pending_halves = 0u;
  s_overflow = 0u;
  s_dma_error = 0u;
  s_total_abs_sum = 0u;
  mic_pdm_reset_result();
  s_result_ready = false;
  s_current_ckstr = ckstr;
  s_autodetect_retry_done = true;
  s_autodetect_probe_ms = HAL_GetTick() + MIC_PDM_AUTODETECT_PROBE_MS;
  mic_pdm_configure_clock();
  mic_pdm_configure_sai(ckstr);
  return mic_pdm_start_hw();
}

static void mic_pdm_process_half(uint32_t start_index)
{
  uint32_t sample_index;
  uint32_t abs_sum = 0u;
  int32_t g_prev = 0;
  int32_t g_prev2 = 0;
  uint32_t energy = 0u;
  int64_t a;
  int64_t b;
  int64_t cross;

  for (sample_index = 0u; sample_index < MIC_PDM_PCM_SAMPLES_PER_HALF; sample_index++) {
    uint32_t word_index = start_index + (sample_index * 4u);
    uint32_t ones = 0u;
    int32_t sample;
    uint32_t abs_sample;
    int64_t goertzel_value;

    ones += (uint32_t)__builtin_popcount((unsigned int)s_raw_buffer[word_index + 0u]);
    ones += (uint32_t)__builtin_popcount((unsigned int)s_raw_buffer[word_index + 1u]);
    ones += (uint32_t)__builtin_popcount((unsigned int)s_raw_buffer[word_index + 2u]);
    ones += (uint32_t)__builtin_popcount((unsigned int)s_raw_buffer[word_index + 3u]);

    sample = (int32_t)ones - 32;
    s_result.last_sample = (int16_t)sample;
    s_result.pcm_samples += 1u;
    abs_sample = (uint32_t)((sample < 0) ? -sample : sample);
    abs_sum += abs_sample;
    if (abs_sample > s_result.peak_abs) {
      s_result.peak_abs = (uint16_t)abs_sample;
    }

    goertzel_value = (int64_t)sample +
                     (((int64_t)MIC_PDM_GOERTZEL_COEFF_Q14 * (int64_t)g_prev) >> 14) -
                     (int64_t)g_prev2;
    g_prev2 = g_prev;
    g_prev = (int32_t)goertzel_value;
  }

  a = (int64_t)g_prev * (int64_t)g_prev;
  b = (int64_t)g_prev2 * (int64_t)g_prev2;
  cross = (((int64_t)MIC_PDM_GOERTZEL_COEFF_Q14 * (int64_t)g_prev) >> 14) * (int64_t)g_prev2;
  if ((a + b) > cross) {
    energy = (uint32_t)((a + b) - cross);
  }

  s_result.block_count += 1u;
  s_result.pdm_words += MIC_PDM_HALF_BUFFER_WORDS;
  s_total_abs_sum += abs_sum;
  s_result.avg_abs = (uint16_t)(s_total_abs_sum / s_result.pcm_samples);
  if (energy > s_result.hf15_energy) {
    s_result.hf15_energy = energy;
  }
}

static void mic_pdm_finalize(void)
{
  bool had_progress = mic_pdm_has_progress();

  s_result.sai_enabled = ((SAI1_Block_A->CR1 & SAI_xCR1_SAIEN) != 0u);
  s_result.dma_enabled = ((SAI1_Block_A->CR1 & SAI_xCR1_DMAEN) != 0u);
  s_result.ckstr = ((SAI1_Block_A->CR1 & SAI_xCR1_CKSTR) != 0u);
  s_result.pdm_enabled = ((SAI1->PDMCR & SAI_PDMCR_PDMEN) != 0u);
  s_result.dma_cndtr = s_mic_dma.Instance->CNDTR;
  s_result.sai_sr = SAI1_Block_A->SR;
  mic_pdm_stop_hw();
  mic_pdm_restore_shared_pins();
  s_result.capture_ms = HAL_GetTick() - s_start_tick_ms;
  s_result.overflow = (s_overflow != 0u);
  s_result.dma_error = (s_dma_error != 0u);
  s_result.autodetect_failed = (!had_progress && s_autodetect_retry_done);
  s_result.valid = (s_result.pcm_samples != 0u) && !s_result.dma_error;
  if (s_result.valid) {
    s_preferred_ckstr = s_current_ckstr;
  }
  s_state = MIC_PDM_STATE_IDLE;
  s_result_ready = true;
}

static void mic_pdm_dma_half_complete(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  if ((s_pending_halves & 0x01u) != 0u) {
    s_overflow = 1u;
  }
  s_pending_halves |= 0x01u;
}

static void mic_pdm_dma_complete(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  if ((s_pending_halves & 0x02u) != 0u) {
    s_overflow = 1u;
  }
  s_pending_halves |= 0x02u;
}

static void mic_pdm_dma_error(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  s_dma_error = 1u;
}

bool mic_pdm_init(void)
{
  if (s_initialized) {
    return true;
  }

  if (!mic_pdm_configure_clock()) {
    return false;
  }

  if (!mic_pdm_configure_dma()) {
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
  s_autodetect_probe_ms = s_start_tick_ms + MIC_PDM_AUTODETECT_PROBE_MS;
  s_total_abs_sum = 0u;
  s_current_ckstr = s_preferred_ckstr;
  s_autodetect_retry_done = false;

  mic_pdm_power_on();
  mic_pdm_configure_shared_pins_for_capture();
  mic_pdm_configure_clock();
  mic_pdm_configure_sai(s_current_ckstr);
  if (!mic_pdm_start_hw()) {
    mic_pdm_restore_shared_pins();
    __HAL_RCC_SAI1_CLK_DISABLE();
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

  pending = s_pending_halves;
  if ((pending & 0x02u) != 0u) {
    s_pending_halves &= (uint8_t)~0x02u;
    mic_pdm_process_half(MIC_PDM_HALF_BUFFER_WORDS);
  }

  if (s_dma_error != 0u) {
    mic_pdm_finalize();
    return;
  }

  if (!s_autodetect_retry_done &&
      mic_pdm_tick_expired(s_autodetect_probe_ms) &&
      !mic_pdm_has_progress()) {
    if (!mic_pdm_restart_with_polarity(!s_current_ckstr)) {
      mic_pdm_finalize();
      return;
    }
  }

  if ((s_state == MIC_PDM_STATE_STOPPING) && mic_pdm_tick_expired(s_stop_after_ms) &&
      (s_pending_halves == 0u)) {
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

void mic_pdm_dma_irqhandler(void)
{
  HAL_DMA_IRQHandler(&s_mic_dma);
}
