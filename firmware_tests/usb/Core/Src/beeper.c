#include "beeper.h"

#include <stddef.h>

#define BEEPER_PWM_PERIOD_TICKS 15u
#define BEEPER_PWM_PULSE_TICKS   8u

static LPTIM_HandleTypeDef *s_timer = NULL;
static uint8_t s_active = 0u;
static uint32_t s_stop_tick_ms = 0u;

void beeper_init(LPTIM_HandleTypeDef *timer)
{
  s_timer = timer;
  s_active = 0u;
  s_stop_tick_ms = 0u;
}

void beeper_play(uint32_t duration_ms)
{
  if (s_timer == NULL) {
    return;
  }

  if (s_active == 0u) {
    if (HAL_LPTIM_PWM_Start(s_timer,
                            BEEPER_PWM_PERIOD_TICKS,
                            BEEPER_PWM_PULSE_TICKS) != HAL_OK) {
      return;
    }
    s_active = 1u;
  }

  s_stop_tick_ms = HAL_GetTick() + duration_ms;
}

void beeper_tick(void)
{
  if ((s_timer == NULL) || (s_active == 0u)) {
    return;
  }

  if ((int32_t)(HAL_GetTick() - s_stop_tick_ms) < 0) {
    return;
  }

  (void)HAL_LPTIM_PWM_Stop(s_timer);
  s_active = 0u;
}

bool beeper_is_active(void)
{
  return (s_active != 0u);
}
