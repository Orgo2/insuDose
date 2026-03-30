#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

bool app_runtime_init(I2C_HandleTypeDef *temp_i2c,
                      LPTIM_HandleTypeDef *beeper_timer);
void app_runtime_tick(void);
void app_runtime_on_exti(uint16_t gpio_pin);
void app_runtime_on_rtc_wakeup(void);
void app_runtime_on_pvd(void);

#endif /* APP_RUNTIME_H */
