#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#include "../../Drivers/RTC/rtc_driver.h"
#include "mic_pdm.h"

bool app_display_init(void);
void app_display_task(void);
bool app_display_is_busy(void);
bool app_display_is_stop2_safe(void);
void app_display_set_usb_session_active(bool active);
void app_display_note_dose(uint8_t dose_units,
                           int8_t temperature_c,
                           int32_t battery_mv,
                           const rtc_datetime_t *datetime,
                           bool datetime_valid);
void app_display_note_mic_debug(const mic_pdm_result_t *result, int32_t battery_mv);

#endif /* APP_DISPLAY_H */
