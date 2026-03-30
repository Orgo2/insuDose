#include "gdem0097t61.h"

#include <stddef.h>

typedef struct
{
  uint8_t command;
  const uint8_t *data;
  uint16_t data_len;
  bool wait_busy_after;
} gdem0097t61_step_t;

typedef enum
{
  GDEM_FLOW_UNINITIALIZED = 0,
  GDEM_FLOW_IDLE,
  GDEM_FLOW_RESET_LOW_WAIT,
  GDEM_FLOW_RESET_HIGH_WAIT,
  GDEM_FLOW_WAIT_BUSY,
  GDEM_FLOW_RUN_SEQUENCE,
  GDEM_FLOW_SLEEP_SETTLE,
  GDEM_FLOW_ERROR
} gdem0097t61_flow_t;

typedef enum
{
  GDEM_SEQUENCE_NONE = 0,
  GDEM_SEQUENCE_INIT,
  GDEM_SEQUENCE_FRAME,
  GDEM_SEQUENCE_UPDATE,
  GDEM_SEQUENCE_SLEEP
} gdem0097t61_sequence_t;

typedef enum
{
  GDEM_REFRESH_FULL = 0,
  GDEM_REFRESH_FAST
} gdem0097t61_refresh_mode_t;

#define GDEM0097T61_RESET_DELAY_MS      10u
#define GDEM0097T61_BUSY_TIMEOUT_MS   3000u
#define GDEM0097T61_SLEEP_SETTLE_MS    100u
#define GDEM0097T61_FAST_REFRESH_LIMIT   4u
#define GDEM0097T61_GUI_RAM_HEIGHT   (GDEM0097T61_HEIGHT + 112u)

static SPI_HandleTypeDef *s_spi = NULL;
static gdem0097t61_flow_t s_flow = GDEM_FLOW_UNINITIALIZED;
static gdem0097t61_sequence_t s_sequence = GDEM_SEQUENCE_NONE;
static gdem0097t61_state_t s_public_state = GDEM0097T61_STATE_UNINITIALIZED;
static gdem0097t61_refresh_mode_t s_refresh_mode = GDEM_REFRESH_FULL;
static const uint8_t *s_framebuffer = NULL;
static uint32_t s_deadline_ms = 0u;
static uint8_t s_step_index = 0u;
static uint8_t s_fast_refresh_count = 0u;
static bool s_force_full_refresh = true;

static const uint8_t s_driver_output_control[] = {
  (uint8_t)((GDEM0097T61_GUI_RAM_HEIGHT - 1u) & 0xFFu),
  (uint8_t)(((GDEM0097T61_GUI_RAM_HEIGHT - 1u) >> 8) & 0xFFu),
  0x01u
};
static const uint8_t s_data_entry_mode[] = { 0x01u };
static const uint8_t s_ram_x_window[] = { 0x00u, (uint8_t)((GDEM0097T61_WIDTH / 8u) - 1u) };
static const uint8_t s_ram_y_window[] = {
  (uint8_t)((GDEM0097T61_GUI_RAM_HEIGHT - 1u) & 0xFFu),
  (uint8_t)(((GDEM0097T61_GUI_RAM_HEIGHT - 1u) >> 8) & 0xFFu),
  0x00u,
  0x00u
};
static const uint8_t s_border_waveform_full[] = { 0x05u };
static const uint8_t s_display_update_control[] = { 0x00u, 0x80u };
static const uint8_t s_temperature_sensor[] = { 0x80u };
static const uint8_t s_ram_x_address[] = { 0x00u };
static const uint8_t s_ram_y_address[] = {
  (uint8_t)((GDEM0097T61_GUI_RAM_HEIGHT - 1u) & 0xFFu),
  (uint8_t)(((GDEM0097T61_GUI_RAM_HEIGHT - 1u) >> 8) & 0xFFu)
};
static const uint8_t s_fast_load_temperature_a[] = { 0xB1u };
static const uint8_t s_fast_write_temperature[] = { 0x64u, 0x00u };
static const uint8_t s_fast_load_temperature_b[] = { 0x91u };
static const uint8_t s_update_control_full[] = { 0xF7u };
static const uint8_t s_update_control_fast[] = { 0xC7u };
static const uint8_t s_sleep_data[] = { 0x01u };

static const gdem0097t61_step_t s_full_init_steps[] = {
  { 0x12u, NULL, 0u, true  },
  { 0x01u, s_driver_output_control, sizeof(s_driver_output_control), false },
  { 0x11u, s_data_entry_mode, sizeof(s_data_entry_mode), false },
  { 0x44u, s_ram_x_window, sizeof(s_ram_x_window), false },
  { 0x45u, s_ram_y_window, sizeof(s_ram_y_window), false },
  { 0x3Cu, s_border_waveform_full, sizeof(s_border_waveform_full), false },
  { 0x21u, s_display_update_control, sizeof(s_display_update_control), false },
  { 0x18u, s_temperature_sensor, sizeof(s_temperature_sensor), false },
  { 0x4Eu, s_ram_x_address, sizeof(s_ram_x_address), false },
  { 0x4Fu, s_ram_y_address, sizeof(s_ram_y_address), true  }
};

static const gdem0097t61_step_t s_fast_init_steps[] = {
  { 0x12u, NULL, 0u, true  },
  { 0x18u, s_temperature_sensor, sizeof(s_temperature_sensor), false },
  { 0x22u, s_fast_load_temperature_a, sizeof(s_fast_load_temperature_a), false },
  { 0x20u, NULL, 0u, true  },
  { 0x1Au, s_fast_write_temperature, sizeof(s_fast_write_temperature), false },
  { 0x22u, s_fast_load_temperature_b, sizeof(s_fast_load_temperature_b), false },
  { 0x20u, NULL, 0u, true  }
};

static bool gdem0097t61_tick_expired(uint32_t deadline_ms)
{
  return ((int32_t)(HAL_GetTick() - deadline_ms) >= 0);
}

static bool gdem0097t61_busy_is_asserted(void)
{
  return (HAL_GPIO_ReadPin(D_busy_GPIO_Port, D_busy_Pin) == GPIO_PIN_SET);
}

static void gdem0097t61_set_idle_pins(void)
{
  HAL_GPIO_WritePin(D_cs_GPIO_Port, D_cs_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(D_dc_GPIO_Port, D_dc_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(D_rst_GPIO_Port, D_rst_Pin, GPIO_PIN_SET);
}

static void gdem0097t61_set_idle_state(void)
{
  s_framebuffer = NULL;
  s_sequence = GDEM_SEQUENCE_NONE;
  s_step_index = 0u;
  s_public_state = GDEM0097T61_STATE_IDLE;
  s_flow = GDEM_FLOW_IDLE;
  gdem0097t61_set_idle_pins();
}

static void gdem0097t61_enter_error(void)
{
  s_framebuffer = NULL;
  s_sequence = GDEM_SEQUENCE_NONE;
  s_step_index = 0u;
  s_force_full_refresh = true;
  s_public_state = GDEM0097T61_STATE_ERROR;
  s_flow = GDEM_FLOW_ERROR;
  gdem0097t61_set_idle_pins();
}

static gdem0097t61_refresh_mode_t gdem0097t61_select_refresh_mode(void)
{
  if (s_force_full_refresh || (s_public_state == GDEM0097T61_STATE_ERROR)) {
    s_force_full_refresh = false;
    s_fast_refresh_count = 0u;
    return GDEM_REFRESH_FULL;
  }

  if (s_fast_refresh_count >= GDEM0097T61_FAST_REFRESH_LIMIT) {
    s_fast_refresh_count = 0u;
    return GDEM_REFRESH_FULL;
  }

  s_fast_refresh_count++;
  return GDEM_REFRESH_FAST;
}

static void gdem0097t61_enter_busy_wait(void)
{
  s_deadline_ms = HAL_GetTick() + GDEM0097T61_BUSY_TIMEOUT_MS;
  s_flow = GDEM_FLOW_WAIT_BUSY;
}

static bool gdem0097t61_write_byte(bool data_mode, uint8_t value)
{
  if (s_spi == NULL) {
    return false;
  }

  HAL_GPIO_WritePin(D_dc_GPIO_Port, D_dc_Pin, data_mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(D_cs_GPIO_Port, D_cs_Pin, GPIO_PIN_RESET);

  if (HAL_SPI_Transmit(s_spi, &value, 1u, HAL_MAX_DELAY) != HAL_OK) {
    HAL_GPIO_WritePin(D_cs_GPIO_Port, D_cs_Pin, GPIO_PIN_SET);
    return false;
  }

  HAL_GPIO_WritePin(D_cs_GPIO_Port, D_cs_Pin, GPIO_PIN_SET);
  return true;
}

static bool gdem0097t61_write_buffer(const uint8_t *buffer, uint16_t length)
{
  uint16_t index = 0u;

  if ((buffer == NULL) || (length == 0u)) {
    return false;
  }

  for (index = 0u; index < length; index++) {
    if (!gdem0097t61_write_byte(true, buffer[index])) {
      return false;
    }
  }

  return true;
}

static bool gdem0097t61_send_step(const gdem0097t61_step_t *step)
{
  if (step == NULL) {
    return false;
  }

  if (!gdem0097t61_write_byte(false, step->command)) {
    return false;
  }

  if (step->data_len == 0u) {
    return true;
  }

  return gdem0097t61_write_buffer(step->data, step->data_len);
}

static void gdem0097t61_advance_sequence(void)
{
  switch (s_sequence) {
    case GDEM_SEQUENCE_INIT:
      s_sequence = GDEM_SEQUENCE_FRAME;
      s_step_index = 0u;
      s_flow = GDEM_FLOW_RUN_SEQUENCE;
      break;

    case GDEM_SEQUENCE_FRAME:
      s_sequence = GDEM_SEQUENCE_UPDATE;
      s_step_index = 0u;
      s_flow = GDEM_FLOW_RUN_SEQUENCE;
      break;

    case GDEM_SEQUENCE_UPDATE:
      s_sequence = GDEM_SEQUENCE_SLEEP;
      s_step_index = 0u;
      s_flow = GDEM_FLOW_RUN_SEQUENCE;
      break;

    case GDEM_SEQUENCE_SLEEP:
      s_deadline_ms = HAL_GetTick() + GDEM0097T61_SLEEP_SETTLE_MS;
      s_flow = GDEM_FLOW_SLEEP_SETTLE;
      break;

    default:
      gdem0097t61_enter_error();
      break;
  }
}

static bool gdem0097t61_get_step(gdem0097t61_step_t *step)
{
  static gdem0097t61_step_t dynamic_step;
  const gdem0097t61_step_t *table = NULL;
  uint8_t table_len = 0u;

  if (step == NULL) {
    return false;
  }

  switch (s_sequence) {
    case GDEM_SEQUENCE_INIT:
      if (s_refresh_mode == GDEM_REFRESH_FAST) {
        table = s_fast_init_steps;
        table_len = (uint8_t)(sizeof(s_fast_init_steps) / sizeof(s_fast_init_steps[0]));
      } else {
        table = s_full_init_steps;
        table_len = (uint8_t)(sizeof(s_full_init_steps) / sizeof(s_full_init_steps[0]));
      }

      if ((table == NULL) || (s_step_index >= table_len)) {
        return false;
      }

      *step = table[s_step_index];
      return true;

    case GDEM_SEQUENCE_FRAME:
      if ((s_step_index != 0u) || (s_framebuffer == NULL)) {
        return false;
      }

      dynamic_step.command = 0x24u;
      dynamic_step.data = s_framebuffer;
      dynamic_step.data_len = GDEM0097T61_FRAME_BYTES;
      dynamic_step.wait_busy_after = false;
      *step = dynamic_step;
      return true;

    case GDEM_SEQUENCE_UPDATE:
      if (s_step_index == 0u) {
        dynamic_step.command = 0x22u;
        dynamic_step.data = (s_refresh_mode == GDEM_REFRESH_FAST) ?
                            s_update_control_fast :
                            s_update_control_full;
        dynamic_step.data_len = sizeof(s_update_control_full);
        dynamic_step.wait_busy_after = false;
        *step = dynamic_step;
        return true;
      }

      if (s_step_index == 1u) {
        dynamic_step.command = 0x20u;
        dynamic_step.data = NULL;
        dynamic_step.data_len = 0u;
        dynamic_step.wait_busy_after = true;
        *step = dynamic_step;
        return true;
      }

      return false;

    case GDEM_SEQUENCE_SLEEP:
      if (s_step_index != 0u) {
        return false;
      }

      dynamic_step.command = 0x10u;
      dynamic_step.data = s_sleep_data;
      dynamic_step.data_len = sizeof(s_sleep_data);
      dynamic_step.wait_busy_after = false;
      *step = dynamic_step;
      return true;

    default:
      return false;
  }
}

static void gdem0097t61_start_refresh(const uint8_t *framebuffer, gdem0097t61_refresh_mode_t mode)
{
  s_framebuffer = framebuffer;
  s_refresh_mode = mode;
  s_sequence = GDEM_SEQUENCE_INIT;
  s_step_index = 0u;
  s_public_state = GDEM0097T61_STATE_BUSY;
  HAL_GPIO_WritePin(D_rst_GPIO_Port, D_rst_Pin, GPIO_PIN_RESET);
  s_deadline_ms = HAL_GetTick() + GDEM0097T61_RESET_DELAY_MS;
  s_flow = GDEM_FLOW_RESET_LOW_WAIT;
}

bool gdem0097t61_init(SPI_HandleTypeDef *spi)
{
  if (spi == NULL) {
    return false;
  }

  s_spi = spi;
  s_flow = GDEM_FLOW_IDLE;
  s_sequence = GDEM_SEQUENCE_NONE;
  s_public_state = GDEM0097T61_STATE_IDLE;
  s_refresh_mode = GDEM_REFRESH_FULL;
  s_framebuffer = NULL;
  s_deadline_ms = 0u;
  s_step_index = 0u;
  s_fast_refresh_count = 0u;
  s_force_full_refresh = true;
  gdem0097t61_set_idle_pins();
  return true;
}

bool gdem0097t61_request_refresh(const uint8_t *framebuffer)
{
  if ((framebuffer == NULL) || (s_spi == NULL)) {
    return false;
  }

  if (gdem0097t61_is_busy()) {
    return false;
  }

  gdem0097t61_start_refresh(framebuffer, gdem0097t61_select_refresh_mode());
  return true;
}

void gdem0097t61_task(void)
{
  gdem0097t61_step_t step;
  bool progress = true;

  while (progress) {
    progress = false;

    switch (s_flow) {
      case GDEM_FLOW_UNINITIALIZED:
      case GDEM_FLOW_IDLE:
      case GDEM_FLOW_ERROR:
        break;

      case GDEM_FLOW_RESET_LOW_WAIT:
        if (gdem0097t61_tick_expired(s_deadline_ms)) {
          HAL_GPIO_WritePin(D_rst_GPIO_Port, D_rst_Pin, GPIO_PIN_SET);
          s_deadline_ms = HAL_GetTick() + GDEM0097T61_RESET_DELAY_MS;
          s_flow = GDEM_FLOW_RESET_HIGH_WAIT;
          progress = true;
        }
        break;

      case GDEM_FLOW_RESET_HIGH_WAIT:
        if (gdem0097t61_tick_expired(s_deadline_ms)) {
          gdem0097t61_enter_busy_wait();
          progress = true;
        }
        break;

      case GDEM_FLOW_WAIT_BUSY:
        if (!gdem0097t61_busy_is_asserted()) {
          s_flow = GDEM_FLOW_RUN_SEQUENCE;
          progress = true;
        } else if (gdem0097t61_tick_expired(s_deadline_ms)) {
          gdem0097t61_enter_error();
        }
        break;

      case GDEM_FLOW_RUN_SEQUENCE:
        if (!gdem0097t61_get_step(&step)) {
          gdem0097t61_advance_sequence();
          progress = true;
          break;
        }

        if (!gdem0097t61_send_step(&step)) {
          gdem0097t61_enter_error();
          break;
        }

        s_step_index++;
        if (step.wait_busy_after) {
          gdem0097t61_enter_busy_wait();
        } else {
          progress = true;
        }
        break;

      case GDEM_FLOW_SLEEP_SETTLE:
        if (gdem0097t61_tick_expired(s_deadline_ms)) {
          gdem0097t61_set_idle_state();
        }
        break;

      default:
        gdem0097t61_enter_error();
        break;
    }
  }
}

void gdem0097t61_on_busy_exti(void)
{
  if ((s_flow == GDEM_FLOW_WAIT_BUSY) && !gdem0097t61_busy_is_asserted()) {
    s_flow = GDEM_FLOW_RUN_SEQUENCE;
  }
}

void gdem0097t61_on_spi_tx_complete(SPI_HandleTypeDef *hspi)
{
  (void)hspi;
}

bool gdem0097t61_is_busy(void)
{
  return (s_flow != GDEM_FLOW_UNINITIALIZED) &&
         (s_flow != GDEM_FLOW_IDLE) &&
         (s_flow != GDEM_FLOW_ERROR);
}

bool gdem0097t61_is_idle(void)
{
  return !gdem0097t61_is_busy();
}

bool gdem0097t61_get_state(gdem0097t61_state_t *state)
{
  if (state == NULL) {
    return false;
  }

  *state = s_public_state;
  return true;
}
