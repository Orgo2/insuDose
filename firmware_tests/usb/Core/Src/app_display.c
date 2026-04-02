#include "app_display.h"

#include <stdio.h>
#include <string.h>

#include "../../Drivers/Charger/charger.h"
#include "../../Drivers/Display/Display_EPD_W21.h"
#include "../../Drivers/Display/Display_EPD_W21_spi.h"
#include "../../Drivers/Display/GUI_Paint.h"
#include "../../Drivers/Display/fonts.h"
#include "mic_pdm.h"
#include "../../USB_Device/Logger/logger.h"

/*
 * E-paper view model.
 * The module caches the last data that should be visible on screen and starts
 * a physical refresh only when the rendered output changed. The EPD driver is
 * asynchronous, so the rest of the firmware can keep running while the panel
 * finishes its internal update.
 */

/* Startup delay before the first visible refresh after the power-on white clear. */
#define APP_DISPLAY_STARTUP_DELAY_MS 1000u
/* Period for re-evaluating elapsed time and USB charge text on screen. */
#define APP_DISPLAY_PERIODIC_REFRESH_MS 60000u
/* Sentinel meaning that elapsed time cannot be computed from RTC data. */
#define APP_DISPLAY_ELAPSED_INVALID  UINT32_MAX
/* Clamp displayed elapsed time to MM:SS-style two-digit hours and minutes. */
#define APP_DISPLAY_ELAPSED_MAX_MIN  ((99u * 60u) + 99u)

/* Raw framebuffer sent to the panel when a new frame is committed. */
static uint8_t s_display_framebuffer[EPD_ARRAY];
/* True after Paint_* helpers were bound to the framebuffer memory. */
static bool s_framebuffer_ready = false;
/* True until the initial all-white panel clear was successfully started. */
static bool s_startup_clear_pending = false;
/* True when cached screen content no longer matches the last rendered frame. */
static bool s_display_dirty = false;
/* True while USB charging/session screen should be shown instead of dose history. */
static bool s_usb_session_active = false;
/* True after at least one valid dose event was recorded for display. */
static bool s_has_last_dose = false;
/* True after a microphone diagnostic result was captured and should be shown. */
static bool s_has_mic_debug = false;
/* Dose value shown in the large left-hand field. */
static uint8_t s_last_dose_units = 0u;
/* Temperature sampled around the last dose event. */
static int8_t s_last_dose_temp_c = LOG_NO_TEMP;
/* Battery voltage sampled around the last dose event. */
static int32_t s_last_dose_battery_mv = -1;
/* True when the last dose also has a valid RTC timestamp. */
static bool s_last_dose_datetime_valid = false;
/* RTC timestamp used to compute elapsed time since the last dose. */
static rtc_datetime_t s_last_dose_datetime = {0};
/* Last microphone capture diagnostics shown on the info line. */
static mic_pdm_result_t s_last_mic_debug = {0};
/* Earliest tick when another display operation may start. */
static uint32_t s_startup_not_before_ms = 0u;
/* Deadline for re-checking elapsed time since last dose. */
static uint32_t s_next_elapsed_refresh_ms = 0u;
/* Deadline for re-checking USB charging text / percentage. */
static uint32_t s_next_usb_refresh_ms = 0u;
/* Last elapsed-minutes value that was really rendered to the panel. */
static uint32_t s_last_rendered_elapsed_minutes = APP_DISPLAY_ELAPSED_INVALID;
/* Last USB charge state label rendered to the panel. */
static int s_last_rendered_usb_status = -1;
/* Last battery percent rendered in USB mode. */
static int s_last_rendered_usb_percent = -1;

static bool app_display_tick_expired(uint32_t deadline_ms);
static bool app_display_is_leap_year(uint16_t year);
static bool app_display_datetime_to_seconds(const rtc_datetime_t *datetime, uint32_t *seconds);
static uint32_t app_display_get_elapsed_minutes(void);
static void app_display_format_battery_at_dose(char *buffer, size_t buffer_len);
static void app_display_format_temp_at_dose(char *buffer, size_t buffer_len);
static void app_display_format_dose(char *buffer, size_t buffer_len);
static void app_display_format_elapsed(char *buffer, size_t buffer_len, uint32_t *elapsed_minutes_out);
static int app_display_get_battery_percent(int32_t battery_mv);
static void app_display_format_usb_charge_state(char *buffer, size_t buffer_len, int *status_out, int *percent_out);
static void app_display_draw_battery_icon(uint16_t x, uint16_t y, uint16_t width, uint16_t height);
static void app_display_render_frame(uint32_t *elapsed_minutes_out);
static void app_display_mark_elapsed_dirty_if_needed(void);
static void app_display_mark_usb_dirty_if_needed(void);
static void app_display_format_mic_debug_line1(char *buffer, size_t buffer_len);

static bool app_display_tick_expired(uint32_t deadline_ms)
{
  return ((int32_t)(HAL_GetTick() - deadline_ms) >= 0);
}

static bool app_display_is_leap_year(uint16_t year)
{
  return (((year % 4u) == 0u) && (((year % 100u) != 0u) || ((year % 400u) == 0u)));
}

static bool app_display_datetime_to_seconds(const rtc_datetime_t *datetime, uint32_t *seconds)
{
  static const uint16_t days_before_month[12] = {
      0u, 31u, 59u, 90u, 120u, 151u, 181u, 212u, 243u, 273u, 304u, 334u
  };
  uint32_t days = 0u;
  uint16_t year;

  if ((datetime == NULL) || (seconds == NULL) ||
      (datetime->date.month < 1u) || (datetime->date.month > 12u) ||
      (datetime->date.day < 1u) || (datetime->date.day > 31u) ||
      (datetime->time.hours > 23u) || (datetime->time.minutes > 59u) ||
      (datetime->time.seconds > 59u)) {
    return false;
  }

  for (year = 2000u; year < datetime->date.year; year++) {
    days += app_display_is_leap_year(year) ? 366u : 365u;
  }

  days += days_before_month[datetime->date.month - 1u];
  if ((datetime->date.month > 2u) && app_display_is_leap_year(datetime->date.year)) {
    days += 1u;
  }
  days += (uint32_t)(datetime->date.day - 1u);

  *seconds = (days * 86400u) +
             ((uint32_t)datetime->time.hours * 3600u) +
             ((uint32_t)datetime->time.minutes * 60u) +
             (uint32_t)datetime->time.seconds;
  return true;
}

static uint32_t app_display_get_elapsed_minutes(void)
{
  rtc_datetime_t now_datetime;
  uint32_t now_seconds;
  uint32_t last_seconds;
  uint32_t elapsed_minutes;

  if (!s_has_last_dose || !s_last_dose_datetime_valid) {
    return APP_DISPLAY_ELAPSED_INVALID;
  }

  if (!rtc_driver_has_valid_datetime() ||
      !rtc_driver_get_datetime(&now_datetime) ||
      !app_display_datetime_to_seconds(&s_last_dose_datetime, &last_seconds) ||
      !app_display_datetime_to_seconds(&now_datetime, &now_seconds) ||
      (now_seconds < last_seconds)) {
    return APP_DISPLAY_ELAPSED_INVALID;
  }

  elapsed_minutes = (now_seconds - last_seconds) / 60u;
  if (elapsed_minutes > APP_DISPLAY_ELAPSED_MAX_MIN) {
    elapsed_minutes = APP_DISPLAY_ELAPSED_MAX_MIN;
  }

  return elapsed_minutes;
}

static void app_display_format_battery_at_dose(char *buffer, size_t buffer_len)
{
  if ((buffer == NULL) || (buffer_len == 0u)) {
    return;
  }

  if (s_last_dose_battery_mv < 0) {
    (void)snprintf(buffer, buffer_len, "BAT --.--V");
    return;
  }

  (void)snprintf(buffer,
                 buffer_len,
                 "BAT %ld.%02ldV",
                 (long)(s_last_dose_battery_mv / 1000),
                 (long)((s_last_dose_battery_mv % 1000) / 10));
}

static void app_display_format_temp_at_dose(char *buffer, size_t buffer_len)
{
  if ((buffer == NULL) || (buffer_len == 0u)) {
    return;
  }

  if (s_last_dose_temp_c == LOG_NO_TEMP) {
    (void)snprintf(buffer, buffer_len, "TMP --C");
    return;
  }

  (void)snprintf(buffer, buffer_len, "TMP %dC", (int)s_last_dose_temp_c);
}

static void app_display_format_dose(char *buffer, size_t buffer_len)
{
  if ((buffer == NULL) || (buffer_len == 0u)) {
    return;
  }

  if (!s_has_last_dose) {
    (void)snprintf(buffer, buffer_len, "--");
    return;
  }

  (void)snprintf(buffer, buffer_len, "%02u", (unsigned)(s_last_dose_units % 100u));
}

static void app_display_format_elapsed(char *buffer, size_t buffer_len, uint32_t *elapsed_minutes_out)
{
  uint32_t elapsed_minutes = app_display_get_elapsed_minutes();

  if ((buffer == NULL) || (buffer_len == 0u)) {
    return;
  }

  if (elapsed_minutes_out != NULL) {
    *elapsed_minutes_out = elapsed_minutes;
  }

  if (elapsed_minutes == APP_DISPLAY_ELAPSED_INVALID) {
    (void)snprintf(buffer, buffer_len, "--:--");
    return;
  }

  (void)snprintf(buffer,
                 buffer_len,
                 "%02lu:%02lu",
                 (unsigned long)(elapsed_minutes / 60u),
                 (unsigned long)(elapsed_minutes % 60u));
}

static int app_display_get_battery_percent(int32_t battery_mv)
{
  int32_t clamped_mv = battery_mv;

  if (battery_mv < 0) {
    return -1;
  }

  if (clamped_mv < (int32_t)CHARGER_BAT_EMPTY_MV) {
    clamped_mv = (int32_t)CHARGER_BAT_EMPTY_MV;
  }
  if (clamped_mv > (int32_t)CHARGER_BAT_CHARGE_STOP_MV) {
    clamped_mv = (int32_t)CHARGER_BAT_CHARGE_STOP_MV;
  }

  return (int)(((clamped_mv - (int32_t)CHARGER_BAT_EMPTY_MV) * 100) /
               ((int32_t)CHARGER_BAT_CHARGE_STOP_MV - (int32_t)CHARGER_BAT_EMPTY_MV));
}

static void app_display_format_usb_charge_state(char *buffer, size_t buffer_len, int *status_out, int *percent_out)
{
  charger_info_t info;
  const char *label = "ERR";
  int percent = -1;
  int status = CHARGER_STATUS_FAULT;

  if ((buffer == NULL) || (buffer_len == 0u)) {
    return;
  }

  memset(&info, 0, sizeof(info));
  charger_get_info(&info);
  percent = app_display_get_battery_percent(info.battery_mv);
  status = info.status;

  if (info.status == CHARGER_STATUS_CHARGING) {
    label = "CHARGING";
  } else if (info.status == CHARGER_STATUS_FULL) {
    label = "CHARGED";
  } else if (info.status == CHARGER_STATUS_IDLE) {
    label = "IDLE";
  } else {
    label = "ERR";
  }

  if (status_out != NULL) {
    *status_out = status;
  }
  if (percent_out != NULL) {
    *percent_out = percent;
  }

  if (percent >= 0) {
    (void)snprintf(buffer, buffer_len, "%s %d%%", label, percent);
  } else {
    (void)snprintf(buffer, buffer_len, "%s --%%", label);
  }
}

static void app_display_draw_battery_icon(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
  uint16_t body_right = (uint16_t)(x + width);
  uint16_t body_bottom = (uint16_t)(y + height);
  uint16_t cap_width = 8u;
  uint16_t cap_height = (uint16_t)(height / 3u);
  uint16_t cap_x = body_right;
  uint16_t cap_y = (uint16_t)(y + ((height - cap_height) / 2u));

  Paint_DrawRectangle((UWORD)x, (UWORD)y, (UWORD)body_right, (UWORD)body_bottom, BLACK, DRAW_FILL_EMPTY, DOT_PIXEL_1X1);
  Paint_DrawRectangle((UWORD)cap_x, (UWORD)cap_y, (UWORD)(cap_x + cap_width), (UWORD)(cap_y + cap_height), BLACK, DRAW_FILL_FULL, DOT_PIXEL_1X1);
}

static void app_display_format_mic_debug_line1(char *buffer, size_t buffer_len)
{
  if ((buffer == NULL) || (buffer_len == 0u)) {
    return;
  }

  if (!s_has_mic_debug) {
    (void)snprintf(buffer, buffer_len, "MIC --");
    return;
  }

  if (!s_last_mic_debug.valid) {
    if (s_last_mic_debug.pdm_words != 0u) {
      (void)snprintf(buffer,
                     buffer_len,
                     "T%u Z%lu F%lu P%u-%u",
                     (unsigned)s_last_mic_debug.diag_stage,
                     (unsigned long)s_last_mic_debug.raw_zero_words,
                     (unsigned long)s_last_mic_debug.raw_full_words,
                     (unsigned)s_last_mic_debug.raw_popcount_min,
                     (unsigned)s_last_mic_debug.raw_popcount_max);
      return;
    }

    (void)snprintf(buffer,
                   buffer_len,
                   "T%uS%uE%02lXC%luN%lu",
                   (unsigned)s_last_mic_debug.diag_stage,
                   (unsigned)s_last_mic_debug.sai_state,
                   (unsigned long)(s_last_mic_debug.sai_error & 0xFFu),
                   (unsigned long)(s_last_mic_debug.sai_clk_hz / 1000000u),
                   (unsigned long)s_last_mic_debug.dma_cndtr);
    return;
  }

  { int32_t cal = mic_pdm_get_bg_offset();
    (void)snprintf(buffer,
                   buffer_len,
                   "P%u A%u C%ld",
                   (unsigned)s_last_mic_debug.peak_abs,
                   (unsigned)s_last_mic_debug.avg_abs,
                   (long)cal);
  }
}

static void app_display_render_frame(uint32_t *elapsed_minutes_out)
{
  char dose_line[8];
  char elapsed_line[8];
  char battery_line[18];
  char temp_line[12];
  char mic_line1[24];
  char usb_status_line[24];
  int usb_status = -1;
  int usb_percent = -1;

  app_display_format_dose(dose_line, sizeof(dose_line));
  app_display_format_elapsed(elapsed_line, sizeof(elapsed_line), elapsed_minutes_out);

  Paint_SelectImage(s_display_framebuffer);
  Paint_Clear(WHITE);

  if (s_usb_session_active) {
    app_display_format_usb_charge_state(usb_status_line, sizeof(usb_status_line), &usb_status, &usb_percent);
    app_display_draw_battery_icon(30u, 12u, 120u, 44u);
    if (usb_percent >= 0) {
      char percent_line[8];
      (void)snprintf(percent_line, sizeof(percent_line), "%d%%", usb_percent);
      Paint_DrawString_EN(58u, 22u, percent_line, &Font20, WHITE, BLACK);
    } else {
      Paint_DrawString_EN(56u, 22u, "--%", &Font20, WHITE, BLACK);
    }
    Paint_DrawString_EN(32u, 66u, usb_status_line, &Font12, WHITE, BLACK);
    if (elapsed_minutes_out != NULL) {
      *elapsed_minutes_out = s_last_rendered_elapsed_minutes;
    }
    s_last_rendered_usb_status = usb_status;
    s_last_rendered_usb_percent = usb_percent;
    return;
  }

  app_display_format_battery_at_dose(battery_line, sizeof(battery_line));
  app_display_format_temp_at_dose(temp_line, sizeof(temp_line));
  app_display_format_mic_debug_line1(mic_line1, sizeof(mic_line1));

  Paint_DrawString_EN(4u, 2u, dose_line, &Font24, WHITE, BLACK);
  Paint_DrawString_EN(76u, 2u, elapsed_line, &Font24, WHITE, BLACK);
  Paint_DrawLine(6u, 34u, 178u, 34u, BLACK, LINE_STYLE_SOLID, DOT_PIXEL_1X1);
  Paint_DrawString_EN(6u, 42u, mic_line1, &Font12, WHITE, BLACK);
  Paint_DrawString_EN(6u, 58u, battery_line, &Font12, WHITE, BLACK);
  Paint_DrawString_EN(100u, 58u, temp_line, &Font12, WHITE, BLACK);

  s_last_rendered_usb_status = -1;
  s_last_rendered_usb_percent = -1;
}

static void app_display_mark_elapsed_dirty_if_needed(void)
{
  uint32_t elapsed_minutes;

  if (!s_has_last_dose || s_display_dirty) {
    return;
  }

  elapsed_minutes = app_display_get_elapsed_minutes();
  if (elapsed_minutes == APP_DISPLAY_ELAPSED_INVALID) {
    return;
  }

  if (elapsed_minutes != s_last_rendered_elapsed_minutes) {
    s_display_dirty = true;
  }
}

static void app_display_mark_usb_dirty_if_needed(void)
{
  char usb_status_line[24];
  int usb_status = -1;
  int usb_percent = -1;

  if (!s_usb_session_active || s_display_dirty) {
    return;
  }

  app_display_format_usb_charge_state(usb_status_line, sizeof(usb_status_line), &usb_status, &usb_percent);
  (void)usb_status_line;
  (void)usb_percent;
  if (usb_status != s_last_rendered_usb_status) {
    s_display_dirty = true;
  }
}

bool app_display_init(void)
{
  /* Own the EPD GPIO block and prepare the framebuffer/painter state once at boot. */
  EPD_GPIO_Init();

  Paint_NewImage(s_display_framebuffer,
                 EPD_WIDTH,
                 EPD_HEIGHT,
                 ROTATE_270,
                 WHITE);
  Paint_SelectImage(s_display_framebuffer);
  Paint_Clear(WHITE);

  s_framebuffer_ready = true;
  s_startup_clear_pending = true;
  s_display_dirty = true;
  s_usb_session_active = false;
  s_has_last_dose = false;
  s_has_mic_debug = false;
  s_last_dose_units = 0u;
  s_last_dose_temp_c = LOG_NO_TEMP;
  s_last_dose_battery_mv = -1;
  s_last_dose_datetime_valid = false;
  memset(&s_last_dose_datetime, 0, sizeof(s_last_dose_datetime));
  memset(&s_last_mic_debug, 0, sizeof(s_last_mic_debug));
  s_startup_not_before_ms = 0u;
  s_next_elapsed_refresh_ms = 0u;
  s_next_usb_refresh_ms = 0u;
  s_last_rendered_elapsed_minutes = APP_DISPLAY_ELAPSED_INVALID;
  s_last_rendered_usb_status = -1;
  s_last_rendered_usb_percent = -1;

  return true;
}

bool app_display_is_busy(void)
{
  return EPD_IsAsyncBusy();
}

bool app_display_is_stop2_safe(void)
{
  return EPD_IsAsyncStop2Safe();
}

void app_display_set_usb_session_active(bool active)
{
  if (s_usb_session_active != active) {
    s_usb_session_active = active;
    s_display_dirty = true;
    s_next_usb_refresh_ms = HAL_GetTick() + APP_DISPLAY_PERIODIC_REFRESH_MS;
    if (!active) {
      s_last_rendered_usb_status = -1;
      s_last_rendered_usb_percent = -1;
    }
  }
}

void app_display_note_dose(uint8_t dose_units,
                           int8_t temperature_c,
                           int32_t battery_mv,
                           const rtc_datetime_t *datetime,
                           bool datetime_valid)
{
  /* Update the cached view model now; the asynchronous task performs the real panel refresh later. */
  s_has_last_dose = true;
  s_last_dose_units = dose_units;
  s_last_dose_temp_c = temperature_c;
  s_last_dose_battery_mv = battery_mv;
  s_last_dose_datetime_valid = false;
  if (datetime_valid && (datetime != NULL)) {
    s_last_dose_datetime = *datetime;
    s_last_dose_datetime_valid = true;
  } else {
    memset(&s_last_dose_datetime, 0, sizeof(s_last_dose_datetime));
  }
  s_next_elapsed_refresh_ms = HAL_GetTick() + APP_DISPLAY_PERIODIC_REFRESH_MS;
  s_last_rendered_elapsed_minutes = APP_DISPLAY_ELAPSED_INVALID;
  s_display_dirty = true;
}

void app_display_note_mic_debug(const mic_pdm_result_t *result, int32_t battery_mv)
{
  /* Mic debug shares the same cached screen model and only marks the frame dirty. */
  if (result == NULL) {
    s_has_mic_debug = false;
    memset(&s_last_mic_debug, 0, sizeof(s_last_mic_debug));
  } else {
    s_has_mic_debug = true;
    s_last_mic_debug = *result;
  }

  (void)battery_mv;

  s_display_dirty = true;
}

/* Pump the asynchronous EPD driver and commit a new frame only when content changed. */
void app_display_task(void)
{
  uint32_t elapsed_minutes = APP_DISPLAY_ELAPSED_INVALID;

  if (!s_framebuffer_ready) {
    return;
  }

  EPD_AsyncTask();

  if (EPD_IsAsyncBusy()) {
    return;
  }

  if (!app_display_tick_expired(s_startup_not_before_ms)) {
    return;
  }

  if (s_startup_clear_pending) {
    if (EPD_StartWhiteScreen_White_Async()) {
      s_startup_clear_pending = false;
      s_startup_not_before_ms = HAL_GetTick() + APP_DISPLAY_STARTUP_DELAY_MS;
    }
    return;
  }

  if (s_usb_session_active) {
    app_display_mark_usb_dirty_if_needed();
    if (app_display_tick_expired(s_next_usb_refresh_ms)) {
      s_display_dirty = true;
      s_next_usb_refresh_ms = HAL_GetTick() + APP_DISPLAY_PERIODIC_REFRESH_MS;
    }
  } else if (s_has_last_dose && app_display_tick_expired(s_next_elapsed_refresh_ms)) {
    app_display_mark_elapsed_dirty_if_needed();
    s_next_elapsed_refresh_ms = HAL_GetTick() + APP_DISPLAY_PERIODIC_REFRESH_MS;
  }

  if (!s_display_dirty) {
    return;
  }

  app_display_render_frame(&elapsed_minutes);
  if (EPD_StartDisplay_Async(s_display_framebuffer)) {
    s_display_dirty = false;
    s_last_rendered_elapsed_minutes = elapsed_minutes;
  }
}
