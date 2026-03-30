#include "rtc_driver.h"

#include "stm32wbxx_hal.h"

#define RTC_DRIVER_VALID_MAGIC    0x52544331u
#define RTC_DRIVER_VALID_BKP_REG  RTC_BKP_DR19
#define RTC_DRIVER_BUILD_BKP_REG  RTC_BKP_DR18

extern RTC_HandleTypeDef hrtc;

static bool rtc_driver_validate_time(const rtc_time_t *time);
static bool rtc_driver_validate_date(const rtc_date_t *date);
static void rtc_driver_fill_default_datetime(rtc_datetime_t *datetime);
static uint32_t rtc_driver_build_signature(void);
static bool rtc_driver_parse_build_datetime(rtc_datetime_t *datetime);
static uint8_t rtc_driver_month_from_string(const char *month);
static uint8_t rtc_driver_weekday_from_date(uint16_t year, uint8_t month, uint8_t day);

static bool rtc_driver_validate_time(const rtc_time_t *time)
{
    if (time == NULL) {
        return false;
    }

    if (time->hours > 23u) {
        return false;
    }
    if (time->minutes > 59u) {
        return false;
    }
    if (time->seconds > 59u) {
        return false;
    }

    return true;
}

static bool rtc_driver_validate_date(const rtc_date_t *date)
{
    if (date == NULL) {
        return false;
    }

    if ((date->year < 2000u) || (date->year > 2099u)) {
        return false;
    }
    if ((date->month < 1u) || (date->month > 12u)) {
        return false;
    }
    if ((date->day < 1u) || (date->day > 31u)) {
        return false;
    }
    if ((date->weekday < 1u) || (date->weekday > 7u)) {
        return false;
    }

    return true;
}

static void rtc_driver_fill_default_datetime(rtc_datetime_t *datetime)
{
    datetime->date.year    = 2000u;
    datetime->date.month   = 1u;
    datetime->date.day     = 1u;
    datetime->date.weekday = 6u;
    datetime->time.hours   = 0u;
    datetime->time.minutes = 0u;
    datetime->time.seconds = 0u;
}

static uint32_t rtc_driver_build_signature(void)
{
    static const char build_id[] = __DATE__ " " __TIME__;
    uint32_t hash = 2166136261u;
    uint32_t i;

    for (i = 0u; build_id[i] != '\0'; i++) {
        hash ^= (uint8_t)build_id[i];
        hash *= 16777619u;
    }

    return hash;
}

static uint8_t rtc_driver_month_from_string(const char *month)
{
    static const char months[12][4] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };
    uint8_t i;

    if (month == NULL) {
        return 0u;
    }

    for (i = 0u; i < 12u; i++) {
        if ((month[0] == months[i][0]) &&
            (month[1] == months[i][1]) &&
            (month[2] == months[i][2])) {
            return (uint8_t)(i + 1u);
        }
    }

    return 0u;
}

static uint8_t rtc_driver_weekday_from_date(uint16_t year, uint8_t month, uint8_t day)
{
    static const uint8_t table[] = {0u, 3u, 2u, 5u, 0u, 3u, 5u, 1u, 4u, 6u, 2u, 4u};
    uint16_t adjusted_year = year;
    uint8_t weekday;

    if (month < 3u) {
        adjusted_year--;
    }

    weekday = (uint8_t)((adjusted_year + (adjusted_year / 4u) - (adjusted_year / 100u) +
                         (adjusted_year / 400u) + table[month - 1u] + day) % 7u);

    return (weekday == 0u) ? 7u : weekday;
}

static bool rtc_driver_parse_build_datetime(rtc_datetime_t *datetime)
{
    const char build_date[] = __DATE__;
    const char build_time[] = __TIME__;
    uint8_t month;
    uint8_t day_tens;

    if (datetime == NULL) {
        return false;
    }

    month = rtc_driver_month_from_string(build_date);
    if (month == 0u) {
        return false;
    }

    day_tens = (build_date[4] == ' ') ? 0u : (uint8_t)(build_date[4] - '0');

    datetime->date.year = (uint16_t)(((uint16_t)(build_date[7] - '0') * 1000u) +
                                     ((uint16_t)(build_date[8] - '0') * 100u) +
                                     ((uint16_t)(build_date[9] - '0') * 10u) +
                                     (uint16_t)(build_date[10] - '0'));
    datetime->date.month = month;
    datetime->date.day = (uint8_t)(day_tens * 10u + (uint8_t)(build_date[5] - '0'));
    datetime->time.hours = (uint8_t)(((uint8_t)(build_time[0] - '0') * 10u) + (uint8_t)(build_time[1] - '0'));
    datetime->time.minutes = (uint8_t)(((uint8_t)(build_time[3] - '0') * 10u) + (uint8_t)(build_time[4] - '0'));
    datetime->time.seconds = (uint8_t)(((uint8_t)(build_time[6] - '0') * 10u) + (uint8_t)(build_time[7] - '0'));
    datetime->date.weekday = rtc_driver_weekday_from_date(datetime->date.year,
                                                          datetime->date.month,
                                                          datetime->date.day);

    return rtc_driver_validate_date(&datetime->date) &&
           rtc_driver_validate_time(&datetime->time);
}

bool rtc_driver_init(void)
{
    return (hrtc.Instance == RTC);
}

bool rtc_driver_set_datetime(const rtc_datetime_t *datetime)
{
    RTC_TimeTypeDef hal_time = {0};
    RTC_DateTypeDef hal_date = {0};

    if (!rtc_driver_init() ||
        (datetime == NULL) ||
        !rtc_driver_validate_date(&datetime->date) ||
        !rtc_driver_validate_time(&datetime->time)) {
        return false;
    }

    hal_time.Hours           = datetime->time.hours;
    hal_time.Minutes         = datetime->time.minutes;
    hal_time.Seconds         = datetime->time.seconds;
    hal_time.DayLightSaving  = RTC_DAYLIGHTSAVING_NONE;
    hal_time.StoreOperation  = RTC_STOREOPERATION_RESET;

    hal_date.WeekDay         = datetime->date.weekday;
    hal_date.Month           = datetime->date.month;
    hal_date.Date            = datetime->date.day;
    hal_date.Year            = (uint8_t)(datetime->date.year % 100u);

    if (HAL_RTC_SetTime(&hrtc, &hal_time, RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }
    if (HAL_RTC_SetDate(&hrtc, &hal_date, RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_DRIVER_VALID_BKP_REG, RTC_DRIVER_VALID_MAGIC);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_DRIVER_BUILD_BKP_REG, rtc_driver_build_signature());
    return true;
}

bool rtc_driver_set_time(const rtc_time_t *time)
{
    rtc_datetime_t datetime;

    if (!rtc_driver_validate_time(time)) {
        return false;
    }

    if (!rtc_driver_get_datetime(&datetime)) {
        rtc_driver_fill_default_datetime(&datetime);
    }

    datetime.time = *time;
    return rtc_driver_set_datetime(&datetime);
}

bool rtc_driver_set_date(const rtc_date_t *date)
{
    rtc_datetime_t datetime;

    if (!rtc_driver_validate_date(date)) {
        return false;
    }

    if (!rtc_driver_get_datetime(&datetime)) {
        rtc_driver_fill_default_datetime(&datetime);
    }

    datetime.date = *date;
    return rtc_driver_set_datetime(&datetime);
}

bool rtc_driver_get_datetime(rtc_datetime_t *datetime)
{
    RTC_TimeTypeDef hal_time = {0};
    RTC_DateTypeDef hal_date = {0};

    if (!rtc_driver_init() || (datetime == NULL)) {
        return false;
    }

    if (HAL_RTC_GetTime(&hrtc, &hal_time, RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }
    if (HAL_RTC_GetDate(&hrtc, &hal_date, RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }

    datetime->time.hours   = hal_time.Hours;
    datetime->time.minutes = hal_time.Minutes;
    datetime->time.seconds = hal_time.Seconds;
    datetime->date.year    = (uint16_t)(2000u + hal_date.Year);
    datetime->date.month   = hal_date.Month;
    datetime->date.day     = hal_date.Date;
    datetime->date.weekday = hal_date.WeekDay;
    return true;
}

bool rtc_driver_get_time(rtc_time_t *time)
{
    rtc_datetime_t datetime;

    if ((time == NULL) || !rtc_driver_get_datetime(&datetime)) {
        return false;
    }

    *time = datetime.time;
    return true;
}

bool rtc_driver_get_date(rtc_date_t *date)
{
    rtc_datetime_t datetime;

    if ((date == NULL) || !rtc_driver_get_datetime(&datetime)) {
        return false;
    }

    *date = datetime.date;
    return true;
}

bool rtc_driver_has_valid_datetime(void)
{
    if (!rtc_driver_init()) {
        return false;
    }

    return (HAL_RTCEx_BKUPRead(&hrtc, RTC_DRIVER_VALID_BKP_REG) == RTC_DRIVER_VALID_MAGIC);
}

bool rtc_driver_sync_build_datetime(void)
{
    rtc_datetime_t build_datetime;
    uint32_t build_signature;

    if (!rtc_driver_init()) {
        return false;
    }

    build_signature = rtc_driver_build_signature();
    if (rtc_driver_has_valid_datetime() &&
        (HAL_RTCEx_BKUPRead(&hrtc, RTC_DRIVER_BUILD_BKP_REG) == build_signature)) {
        return true;
    }

    if (!rtc_driver_parse_build_datetime(&build_datetime)) {
        return false;
    }

    return rtc_driver_set_datetime(&build_datetime);
}

uint32_t rtc_driver_get_fattime(void)
{
    rtc_datetime_t datetime;

    if (!rtc_driver_has_valid_datetime() || !rtc_driver_get_datetime(&datetime)) {
        return 0u;
    }
    if (datetime.date.year < 1980u) {
        return 0u;
    }

    return ((uint32_t)(datetime.date.year - 1980u) << 25) |
           ((uint32_t)datetime.date.month << 21) |
           ((uint32_t)datetime.date.day << 16) |
           ((uint32_t)datetime.time.hours << 11) |
           ((uint32_t)datetime.time.minutes << 5) |
           ((uint32_t)datetime.time.seconds / 2u);
}

bool rtc_driver_start_wakeup(uint32_t period_seconds)
{
    if (!rtc_driver_init()) {
        return false;
    }
    if ((period_seconds == 0u) || (period_seconds > 65536u)) {
        return false;
    }

    if (HAL_RTCEx_DeactivateWakeUpTimer(&hrtc) != HAL_OK) {
        return false;
    }

    return (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,
                                        period_seconds - 1u,
                                        RTC_WAKEUPCLOCK_CK_SPRE_16BITS) == HAL_OK);
}

void rtc_driver_stop_wakeup(void)
{
    if (!rtc_driver_init()) {
        return;
    }

    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
}

void rtc_driver_wakeup_irq_handler(void)
{
    // Pri HAL obsluhe prerusenia netreba robit nic navyse.
}
