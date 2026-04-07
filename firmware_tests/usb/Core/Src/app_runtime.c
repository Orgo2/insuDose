#include "app_runtime.h"

#include "app_display.h"
#include "beeper.h"
#include "mic_pdm.h"
#include "usb_device.h"

#include "../../Drivers/Charger/charger.h"
#include "../../Drivers/RTC/rtc_driver.h"
#include "../../Drivers/TMP102/tmp102.h"
#include "../../USB_Device/Logger/logger.h"
#include "../../USB_Device/Target/usbd_conf.h"

/*
 * Central application runtime.
 * This file ties together charger, temperature sensing, microphone capture,
 * display refresh, USB mass storage and low-power entry. IRQ callbacks only
 * mark events; app_runtime_tick() performs the real work in one place.
 */

/* RTC wakeup period in seconds. This defines how often the low-power loop gets
 * a guaranteed periodic wake event even when no GPIO interrupt occurs.
 */
#define APP_RUNTIME_RTC_WAKEUP_PERIOD_S 1u
/* Maximum time to wait for mic capture before processing dose without mic data. */
#define APP_DOSE_MIC_TIMEOUT_MS         2000u
/* Minimum hold time before a falling Dose edge is treated as a real release, not bounce. */
#define APP_DOSE_DEBOUNCE_MS            100u

typedef enum
{
  SYS_RUNNING = 0,
  SYS_USB_ACTIVE
} app_sys_state_t;

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);

/* Flags changed from callbacks and consumed in the cooperative main loop. */
static volatile app_sys_state_t s_state = SYS_RUNNING;
/* Cached external power/VIN presence sampled from the power-detect GPIO. */
static volatile uint8_t s_power_detect_present = 0u;
/* Latched request to create one dose log entry after a full button cycle. */
static volatile uint8_t s_dose_log_pending = 0u;
/* Latched request to start microphone capture on dose edge. */
static volatile uint8_t s_dose_mic_start_pending = 0u;
/* Latched request to stop microphone capture after the dose edge is complete. */
static volatile uint8_t s_dose_mic_stop_pending = 0u;
/* Periodic or PVD-triggered wake request to refresh battery state. */
static volatile uint8_t s_wake_lowbatt = 0u;
/* Tracks whether the dose button is currently inside a valid press/release cycle. */
static volatile uint8_t s_dose_cycle_armed = 0u;
/* HAL tick when s_dose_log_pending was set; used to timeout stuck mic capture. */
static volatile uint32_t s_dose_log_request_tick = 0u;
/* HAL tick when Dose 0→1 edge was detected; used for debounce on the 1→0 edge. */
static volatile uint32_t s_dose_arm_tick = 0u;

/* True while the host actively owns the RAM disk over USB MSC. */
static bool s_usb_session_active = false;
/* True after MX_USB_Device_Init() brought up the USB stack once. */
static bool s_usb_stack_started = false;
/* Result of mic_pdm_init(); gates all microphone-specific paths. */
static bool s_mic_available = false;
/* True when the last completed microphone capture result is still waiting to be shown. */
static bool s_pending_mic_result_valid = false;
/* Cached result of the most recent microphone capture around a dose event. */
static mic_pdm_result_t s_pending_mic_result = {0};

/* Mic background calibration (triggered by Enter button hold). */
typedef enum {
  MIC_CAL_IDLE = 0,
  MIC_CAL_REQUESTED,       /* Enter pressed, waiting to start capture */
  MIC_CAL_WARMUP,          /* first 1000 ms: capture running, data discarded */
  MIC_CAL_MEASURING,       /* next 1000 ms: averaging the CIC output */
  MIC_CAL_DONE
} mic_cal_state_t;
static mic_cal_state_t s_mic_cal_state = MIC_CAL_IDLE;
static uint32_t s_mic_cal_phase_start = 0u;

static uint8_t app_read_power_detect(void);
static bool app_is_power_present(void);
static bool app_is_usb_session_active(void);
static bool app_is_mic_capture_active(void);
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

/* High-level startup after CubeMX created the raw HAL handles and clocks. */
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
  s_mic_available = mic_pdm_init();

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

/* One cooperative scheduler step: consume events, run services, then choose sleep depth. */
void app_runtime_tick(void)
{
  mic_pdm_result_t mic_result;

  app_update_power_detect();
  beeper_tick();

  if (s_wake_lowbatt != 0u) {
    if (!app_is_mic_capture_active()) {
      s_wake_lowbatt = 0u;
      charger_force_measure();
    }
  }

  if (s_mic_available && (s_dose_mic_start_pending != 0u) &&
      !s_usb_session_active && !mic_pdm_is_active()) {
    if (mic_pdm_start()) {
      s_dose_mic_start_pending = 0u;
    } else {
      /* Mic is only a sidecar; a failed start must never block the dose flow. */
      s_dose_mic_start_pending = 0u;
      s_dose_mic_stop_pending = 0u;
    }
  }

  if (s_mic_available && (s_dose_mic_stop_pending != 0u) && mic_pdm_is_active()) {
    s_dose_mic_stop_pending = 0u;
    mic_pdm_request_stop();
  }

  if (s_mic_available && (s_dose_mic_stop_pending != 0u) &&
      (s_dose_mic_start_pending == 0u) && !mic_pdm_is_active()) {
    s_dose_mic_stop_pending = 0u;
  }

  if (s_mic_available) {
    mic_pdm_task();
    if (mic_pdm_take_result(&mic_result)) {
      if (s_mic_cal_state == MIC_CAL_DONE) {
        /* Calibration capture finished — use signed average as DC offset,
         * compute Goertzel noise threshold from bin magnitudes (mean + 3σ).
         */
        mic_pdm_set_bg_offset(mic_result.avg_signed);
        mic_pdm_finish_goertzel_cal();
        /* Log calibration result: spike detector background energy. */
        {
          uint16_t cal_vals[5];
          cal_vals[0] = (uint16_t)mic_result.scan_target_mag;  /* bg_rms */
          cal_vals[1] = (uint16_t)mic_result.scan_peak_mag;    /* peak_rms */
          cal_vals[2] = (uint16_t)mic_result.scan_peak_k;      /* ratio */
          cal_vals[3] = (uint16_t)mic_result.goertzel_bins_active; /* spikes_3x */
          cal_vals[4] = (uint16_t)mic_result.spike_clusters;   /* clusters */
          append_log_scan(cal_vals, cal_vals, 5u,
                          mic_result.goertzel_bins_total, "CAL");
        }
        /* Log post-capture FFT spectrum for calibration capture. */
        {
          uint16_t fft_mags[MIC_PDM_FFT_BINS];
          uint32_t fft_count = 0u;
          if (mic_pdm_get_fft_spectrum(fft_mags, &fft_count)) {
            uint32_t half = (fft_count > 16u) ? 16u : fft_count;
            append_log_scan(fft_mags, fft_mags, half, fft_count, "CFL");
            if (fft_count > 16u) {
              append_log_scan(fft_mags + 16, fft_mags + 16,
                              fft_count - 16u, fft_count, "CFH");
            }
          }
        }
        s_pending_mic_result = mic_result;
        s_pending_mic_result_valid = true;
        s_mic_cal_state = MIC_CAL_IDLE;
        /* Immediately show calibration threshold on display. */
        app_display_note_mic_debug(&mic_result, -1);
      } else {
        s_pending_mic_result = mic_result;
        s_pending_mic_result_valid = true;
      }
    }
  }

  /* --- Mic background calibration state machine --- */
  switch (s_mic_cal_state) {
  case MIC_CAL_REQUESTED:
    /* Clear any previous offset so the measurement capture is raw. */
    mic_pdm_set_bg_offset(0);
    if (mic_pdm_start()) {
      s_mic_cal_phase_start = HAL_GetTick();
      s_mic_cal_state = MIC_CAL_WARMUP;
    } else {
      s_mic_cal_state = MIC_CAL_IDLE;
    }
    break;

  case MIC_CAL_WARMUP:
    /* After 100 ms, button click noise has settled; start Goertzel calibration.
     * Mic wakeup transient (25 ms) is already discarded internally by the driver.
     */
    if ((int32_t)(HAL_GetTick() - s_mic_cal_phase_start) >= 100) {
      mic_pdm_start_goertzel_cal();
      s_mic_cal_phase_start = HAL_GetTick();
      s_mic_cal_state = MIC_CAL_MEASURING;
    }
    break;

  case MIC_CAL_MEASURING:
    /* After 15 ms, ~11 Goertzel bins collected (N=64, ~1.37 ms/bin); stop. */
    if ((int32_t)(HAL_GetTick() - s_mic_cal_phase_start) >= 15) {
      mic_pdm_request_stop();
      s_mic_cal_state = MIC_CAL_DONE;
    }
    break;

  case MIC_CAL_DONE:
    /* Waiting for mic_pdm_task() to finalize and take_result above to grab it. */
    break;

  case MIC_CAL_IDLE:
  default:
    break;
  }

  if (!app_is_mic_capture_active()) {
    charger_task(app_is_power_present());
    tmp102_set_enabled(!charger_should_sleep());
    tmp102_task();
  }

  app_usb_state_tick();
  app_buttons_tick();
  if (!app_is_mic_capture_active()) {
    app_display_task();
  }

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

/* EXTI callbacks are translated into application events and deferred state changes. */
void app_runtime_on_exti(uint16_t gpio_pin)
{
  if (gpio_pin == Power_Detect_Pin) {
    s_power_detect_present = app_read_power_detect();
    return;
  }

  if (gpio_pin == Dose_Pin) {
    if (HAL_GPIO_ReadPin(Dose_GPIO_Port, Dose_Pin) == GPIO_PIN_SET) {
      s_dose_arm_tick = HAL_GetTick();
      s_dose_cycle_armed = 1u;
      s_pending_mic_result_valid = false;
      memset(&s_pending_mic_result, 0, sizeof(s_pending_mic_result));
      if (s_mic_available && !s_usb_session_active) {
        tmp102_set_enabled(false);
        s_dose_mic_start_pending = 1u;
        s_dose_mic_stop_pending = 0u;
      }
    } else if (s_dose_cycle_armed != 0u) {
      if ((int32_t)(HAL_GetTick() - s_dose_arm_tick) < (int32_t)APP_DOSE_DEBOUNCE_MS) {
        return;
      }
      s_dose_cycle_armed = 0u;
      s_dose_log_pending = 1u;
      s_dose_log_request_tick = HAL_GetTick();
      if (s_mic_available &&
          ((s_dose_mic_start_pending != 0u) || mic_pdm_is_active())) {
        s_dose_mic_stop_pending = 1u;
      }
    }
    return;
  }

  if (gpio_pin == Temp_Alert_Pin) {
    tmp102_notify_alert_irq();
  }

  if (gpio_pin == Enter_Pin) {
    /* Only trigger calibration on press (rising edge) when idle. */
    if ((s_mic_cal_state == MIC_CAL_IDLE) && s_mic_available &&
        !s_usb_session_active && !mic_pdm_is_active() &&
        (s_dose_mic_start_pending == 0u)) {
      s_mic_cal_state = MIC_CAL_REQUESTED;
    }
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

static bool app_is_mic_capture_active(void)
{
  return s_mic_available &&
         ((s_dose_mic_start_pending != 0u) ||
          mic_pdm_is_active());
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

/* Decide whether the system may drop to STOP2 or must stay in shallow sleep/WFI. */
static void app_sleep_until_event(void)
{
  if (s_usb_session_active || s_usb_stack_started || beeper_is_active() ||
      (s_mic_available && mic_pdm_is_active())) {
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

/* Recover the RAM disk and TMP102 metadata from flash only when power conditions allow it. */
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

/* Choose storage source at boot: keep valid RAM disk, restore snapshot or create a new one. */
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

/* Ensure the RAM disk is mounted and coherent before USB exposes it to the host. */
static bool app_prepare_ramdisk_for_usb(void)
{
  if (logger_ramdisk_is_empty()) {
    (void)app_restore_ramdisk_if_allowed();
  }

  return logger_mount_ramdisk();
}

/* Store the live log and temperature history into flash before battery-empty sleep. */
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

/* One-time USB stack startup; later code only tracks whether the host session is active. */
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

/* Translate low-level USB link state into application ownership of the RAM disk and UI. */
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

/* Finalize one logical dose event once all related sensor/mic state is available. */
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

  if (s_usb_session_active) {
    s_dose_log_pending = 0u;
    beeper_play(40u);
    return;
  }

  if (app_is_mic_capture_active()) {
    if ((int32_t)(HAL_GetTick() - s_dose_log_request_tick) < (int32_t)APP_DOSE_MIC_TIMEOUT_MS) {
      return;
    }
    /* Mic timed out — force-stop and finalize immediately so diagnostic
     * result is available for display even when capture produced no data.
     */
    s_dose_mic_start_pending = 0u;
    s_dose_mic_stop_pending = 0u;
    mic_pdm_force_stop();
    /* Grab whatever result (possibly invalid) the forced finalize produced. */
    if (s_mic_available) {
      mic_pdm_result_t timeout_result;
      if (mic_pdm_take_result(&timeout_result)) {
        s_pending_mic_result = timeout_result;
        s_pending_mic_result_valid = true;
      }
    }
  }

  s_dose_log_pending = 0u;

  memset(&charger_info, 0, sizeof(charger_info));
  charger_get_info(&charger_info);
  (void)tmp102_get_last_temperature_rounded(&logged_temp);
  datetime_valid = rtc_driver_has_valid_datetime() && rtc_driver_get_datetime(&dose_datetime);
  log_written = append_log_rotating(dose_units, logged_temp);
  (void)log_written;

  /* Write spike detector metrics to log file. */
  if (s_pending_mic_result_valid) {
    uint16_t spike_vals[5];
    spike_vals[0] = (uint16_t)s_pending_mic_result.scan_target_mag;  /* bg_rms */
    spike_vals[1] = (uint16_t)s_pending_mic_result.scan_peak_mag;    /* peak_rms */
    spike_vals[2] = (uint16_t)s_pending_mic_result.scan_peak_k;      /* ratio */
    spike_vals[3] = (uint16_t)s_pending_mic_result.goertzel_bins_active; /* spikes_3x */
    spike_vals[4] = (uint16_t)s_pending_mic_result.spike_clusters;   /* clusters */
    append_log_scan(spike_vals, spike_vals, 5u,
                    s_pending_mic_result.goertzel_bins_total, "MES");
    /* Log post-capture FFT spectrum for dose measurement. */
    {
      uint16_t fft_mags[MIC_PDM_FFT_BINS];
      uint32_t fft_count = 0u;
      if (mic_pdm_get_fft_spectrum(fft_mags, &fft_count)) {
        uint32_t half = (fft_count > 16u) ? 16u : fft_count;
        append_log_scan(fft_mags, fft_mags, half, fft_count, "FL");
        if (fft_count > 16u) {
          append_log_scan(fft_mags + 16, fft_mags + 16,
                          fft_count - 16u, fft_count, "FH");
        }
      }
    }
    /* Temporary debug: EW value = 10 * EMA energy / background energy.
     * Threshold guide: 15 = re-arm, 30 = detect.
     */
    {
      const uint16_t *env = NULL;
      uint32_t env_count = 0u;
      if (mic_pdm_get_envelope_trace(&env, &env_count)) {
        const uint32_t chunk_size = 32u;
        uint32_t offset = 0u;
        uint32_t chunk_idx = 0u;
        while (offset < env_count) {
          uint32_t n = env_count - offset;
          char label[5];
          if (n > chunk_size) {
            n = chunk_size;
          }
          label[0] = 'E';
          label[1] = 'W';
          if (chunk_idx < 10u) {
            label[2] = (char)('0' + chunk_idx);
            label[3] = '\0';
          } else {
            label[2] = (char)('0' + (chunk_idx / 10u));
            label[3] = (char)('0' + (chunk_idx % 10u));
            label[4] = '\0';
          }
          append_log_scan(env + offset, env + offset, n, env_count, label);
          offset += n;
          chunk_idx++;
        }
      }
    }
  }

  app_display_note_dose(dose_units,
                        logged_temp,
                        charger_info.battery_valid ? charger_info.battery_mv : -1,
                        datetime_valid ? &dose_datetime : NULL,
                        datetime_valid);
  if (s_pending_mic_result_valid) {
    app_display_note_mic_debug(&s_pending_mic_result,
                               charger_info.battery_valid ? charger_info.battery_mv : -1);
  }
}
