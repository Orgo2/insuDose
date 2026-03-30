#include "app_runtime.h"

#include "app_display.h"
#include "beeper.h"
#include "usb_device.h"

#include "../../Drivers/Charger/charger.h"
#include "../../Drivers/RTC/rtc_driver.h"
#include "../../Drivers/TMP102/tmp102.h"
#include "../../USB_Device/Logger/logger.h"
#include "../../USB_Device/Target/usbd_conf.h"

#define APP_RUNTIME_RTC_WAKEUP_PERIOD_S 1u

typedef enum
{
  SYS_RUNNING = 0,
  SYS_USB_ACTIVE
} app_sys_state_t;

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);

static volatile app_sys_state_t s_state = SYS_RUNNING;
static volatile uint8_t s_power_detect_present = 0u;
static volatile uint8_t s_dose_log_pending = 0u;
static volatile uint8_t s_wake_lowbatt = 0u;
static volatile uint8_t s_dose_cycle_armed = 0u;

static bool s_usb_session_active = false;
static bool s_usb_stack_started = false;

static uint8_t app_read_power_detect(void);
static bool app_is_power_present(void);
static bool app_is_usb_session_active(void);
static void app_pvd_init(void);
static void app_rr_start(void);
static void app_rr_stop(void);
static void app_usb_session_enter(void);
static void app_usb_session_leave(void);
static void app_sleep_until_event(void);
static void app_update_power_detect(void);
static bool app_usb_stack_start(void);
static void app_usb_state_tick(void);
static void app_buttons_tick(void);
static bool app_restore_ramdisk_if_allowed(void);
static void app_ensure_ramdisk_ready(void);
static bool app_prepare_ramdisk_for_usb(void);
static void app_snapshot_ramdisk_for_sleep(void);

bool app_runtime_init(I2C_HandleTypeDef *temp_i2c,
                      LPTIM_HandleTypeDef *beeper_timer)
{
  if (!rtc_driver_init()) {
    return false;
  }

  if (!rtc_driver_sync_build_datetime()) {
    return false;
  }

  if (!rtc_driver_start_wakeup(APP_RUNTIME_RTC_WAKEUP_PERIOD_S)) {
    return false;
  }

  if (!tmp102_init(temp_i2c)) {
    return false;
  }

  beeper_init(beeper_timer);
  charger_init();
  app_pvd_init();
  app_ensure_ramdisk_ready();

  if (!app_display_init()) {
    return false;
  }

  s_power_detect_present = app_read_power_detect();
  s_usb_stack_started = false;
  app_usb_session_leave();

  charger_task(app_is_power_present());
  tmp102_set_enabled(true);
  tmp102_task();

  if (!app_usb_stack_start()) {
    return false;
  }

  if (app_is_usb_session_active()) {
    app_usb_session_enter();
    s_state = SYS_USB_ACTIVE;
    return true;
  }

  s_state = SYS_RUNNING;
  if (!logger_ramdisk_is_empty()) {
    app_rr_start();
  }

  return true;
}

void app_runtime_tick(void)
{
  app_update_power_detect();
  beeper_tick();

  if (s_wake_lowbatt != 0u) {
    s_wake_lowbatt = 0u;
    charger_force_measure();
  }

  charger_task(app_is_power_present());
  tmp102_set_enabled(!charger_should_sleep());
  tmp102_task();

  app_usb_state_tick();
  app_buttons_tick();
  app_display_task();

  if ((s_state == SYS_RUNNING) && logger_ramdisk_is_empty()) {
    if (app_restore_ramdisk_if_allowed()) {
      app_rr_start();
    }
  }

  if (charger_should_sleep()) {
    app_rr_stop();
    app_snapshot_ramdisk_for_sleep();
    app_sleep_until_event();
    return;
  }

  app_sleep_until_event();
}

void app_runtime_on_exti(uint16_t gpio_pin)
{
  if (gpio_pin == Power_Detect_Pin) {
    s_power_detect_present = app_read_power_detect();
    return;
  }

  if (gpio_pin == Dose_Pin) {
    if (HAL_GPIO_ReadPin(Dose_GPIO_Port, Dose_Pin) != GPIO_PIN_RESET) {
      s_dose_cycle_armed = 1u;
    } else if (s_dose_cycle_armed != 0u) {
      s_dose_cycle_armed = 0u;
      s_dose_log_pending = 1u;
    }
    return;
  }

  if (gpio_pin == Temp_Alert_Pin) {
    tmp102_notify_alert_irq();
  }
}

void app_runtime_on_rtc_wakeup(void)
{
  s_wake_lowbatt = 1u;
}

void app_runtime_on_pvd(void)
{
  s_wake_lowbatt = 1u;
}

static uint8_t app_read_power_detect(void)
{
  return (HAL_GPIO_ReadPin(Power_Detect_GPIO_Port, Power_Detect_Pin) != GPIO_PIN_RESET) ? 1u : 0u;
}

static bool app_is_power_present(void)
{
  return ((s_power_detect_present != 0u) || app_is_usb_session_active());
}

static void app_pvd_init(void)
{
}

static void app_rr_start(void)
{
}

static void app_rr_stop(void)
{
}

static bool app_is_usb_session_active(void)
{
  if (!s_usb_stack_started || !usbd_link_is_connected()) {
    return false;
  }

  return !usbd_link_is_suspended();
}

static void app_usb_session_enter(void)
{
  s_usb_session_active = true;
  logger_set_usb_session_active(true);
  app_display_set_usb_session_active(true);
}

static void app_usb_session_leave(void)
{
  s_usb_session_active = false;
  logger_set_usb_session_active(false);
  app_display_set_usb_session_active(false);
}

static void app_sleep_until_event(void)
{
  if (s_usb_session_active || s_usb_stack_started || beeper_is_active()) {
    __WFI();
    return;
  }

  if (app_display_is_busy() && !app_display_is_stop2_safe()) {
    __WFI();
    return;
  }

  HAL_SuspendTick();
  HAL_PWREx_EnterSTOP2Mode(PWR_SLEEPENTRY_WFI);
  SystemClock_Config();
  PeriphCommonClock_Config();
  HAL_ResumeTick();
  charger_force_measure();
}

static void app_update_power_detect(void)
{
  s_power_detect_present = app_read_power_detect();
}

static bool app_restore_ramdisk_if_allowed(void)
{
  tmp102_backup_t tmp102_backup;
  uint16_t metadata_len = 0u;

  if (!logger_ramdisk_is_empty()) {
    return logger_mount_ramdisk();
  }

  if (!logger_flash_snapshot_is_available()) {
    return false;
  }

  if (!charger_should_restore()) {
    return false;
  }

  if (!logger_restore_ramdisk_from_flash()) {
    return false;
  }

  if (logger_get_snapshot_metadata(&tmp102_backup,
                                   (uint16_t)sizeof(tmp102_backup),
                                   &metadata_len) &&
      (metadata_len == sizeof(tmp102_backup))) {
    (void)tmp102_import_backup(&tmp102_backup);
  }

  return logger_mount_ramdisk();
}

static void app_ensure_ramdisk_ready(void)
{
  if (!logger_ramdisk_is_empty()) {
    if (logger_mount_ramdisk()) {
      return;
    }
  }

  if (app_restore_ramdisk_if_allowed()) {
    return;
  }

  if (!logger_flash_snapshot_is_available()) {
    (void)init_ramdisk();
  }
}

static bool app_prepare_ramdisk_for_usb(void)
{
  if (logger_ramdisk_is_empty()) {
    (void)app_restore_ramdisk_if_allowed();
  }

  return logger_mount_ramdisk();
}

static void app_snapshot_ramdisk_for_sleep(void)
{
  tmp102_backup_t tmp102_backup;

  tmp102_export_backup(&tmp102_backup);
  logger_set_snapshot_metadata(&tmp102_backup, (uint16_t)sizeof(tmp102_backup));
  tmp102_set_enabled(false);

  if (!logger_ramdisk_is_empty() && logger_persist_ramdisk_to_flash()) {
    logger_invalidate_ramdisk();
  }
}

static bool app_usb_stack_start(void)
{
  if (s_usb_stack_started) {
    return true;
  }

  if (!app_prepare_ramdisk_for_usb()) {
    return false;
  }

  usbd_link_state_reset();
  MX_USB_Device_Init();
  s_usb_stack_started = true;
  return true;
}

static void app_usb_state_tick(void)
{
  bool usb_active_now = app_is_usb_session_active();

  if (usb_active_now) {
    if (!s_usb_session_active) {
      app_rr_stop();
      app_usb_session_enter();
    }
    s_state = SYS_USB_ACTIVE;
    return;
  }

  if (s_usb_session_active) {
    app_usb_session_leave();
    if (app_restore_ramdisk_if_allowed()) {
      app_rr_start();
    }
  }

  s_state = SYS_RUNNING;
}

static void app_buttons_tick(void)
{
  charger_info_t charger_info;
  rtc_datetime_t dose_datetime;
  int8_t logged_temp = LOG_NO_TEMP;
  bool datetime_valid = false;
  bool log_written = false;
  const uint8_t dose_units = 1u;

  if (s_dose_log_pending == 0u) {
    return;
  }

  s_dose_log_pending = 0u;

  if (s_usb_session_active) {
    beeper_play(40u);
    return;
  }

  memset(&charger_info, 0, sizeof(charger_info));
  charger_get_info(&charger_info);
  (void)tmp102_read_temperature_now_rounded(&logged_temp);
  datetime_valid = rtc_driver_has_valid_datetime() && rtc_driver_get_datetime(&dose_datetime);
  log_written = append_log_rotating(dose_units, logged_temp);
  (void)log_written;
  app_display_note_dose(dose_units,
                        logged_temp,
                        charger_info.battery_valid ? charger_info.battery_mv : -1,
                        datetime_valid ? &dose_datetime : NULL,
                        datetime_valid);
  app_display_note_mic_debug(NULL, -1);
}
