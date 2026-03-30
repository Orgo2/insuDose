#include "tmp102.h"

#include <limits.h>
#include <string.h>

// TMP102 pointer registre.
#define TMP102_REG_TEMPERATURE          0x00u
#define TMP102_REG_CONFIGURATION        0x01u
#define TMP102_REG_TLOW                 0x02u
#define TMP102_REG_THIGH                0x03u

// Konfiguracia:
// - 12-bit rozlisenie
// - comparator mode
// - active low ALERT
// - continuous conversion
// - conversion rate 0.25 Hz
#define TMP102_CONFIG_MSB               0x60u
#define TMP102_CONFIG_LSB               0x00u

typedef struct
{
    I2C_HandleTypeDef *i2c;
    bool initialized;
    bool enabled;
    bool powered;
    bool configured;
    bool temperature_valid;
    bool alarm_active;
    bool max_temperature_valid;
    bool overtemp_start_valid;
    bool last_overtemp_end_valid;
    bool alert_irq_pending;
    int16_t temperature_dC;
    int16_t max_temperature_dC;
    uint32_t power_on_tick_ms;
    uint32_t last_idle_sample_tick_ms;
    uint32_t next_retry_tick_ms;
    uint32_t overtemp_completed_s;
    uint32_t last_overtemp_sample_s;
    rtc_datetime_t max_temperature_datetime;
    rtc_datetime_t overtemp_start_datetime;
    rtc_datetime_t last_overtemp_end_datetime;
} tmp102_context_t;

static tmp102_context_t s_tmp102 = {0};

static bool tmp102_write_register(uint8_t reg, const uint8_t *data, uint16_t len);
static bool tmp102_read_register(uint8_t reg, uint8_t *data, uint16_t len);
static int16_t tmp102_raw_to_dC(uint16_t raw);
static uint16_t tmp102_dC_to_raw(int16_t temperature_dC);
static bool tmp102_configure_sensor(void);
static bool tmp102_read_temperature_internal(int16_t *temperature_dC);
static bool tmp102_alarm_pin_is_active(void);
static bool tmp102_datetime_to_seconds(const rtc_datetime_t *datetime, uint32_t *seconds);
static bool tmp102_is_leap_year(uint16_t year);
static uint32_t tmp102_get_active_overtemp_seconds(void);
static bool tmp102_get_now_seconds(uint32_t *seconds, rtc_datetime_t *datetime);
static void tmp102_update_max_temperature(int16_t temperature_dC, const rtc_datetime_t *datetime, bool datetime_valid);
static void tmp102_handle_alarm_assert(void);
static void tmp102_handle_alarm_deassert(void);
static bool tmp102_tick_due(uint32_t now, uint32_t deadline);
static int8_t tmp102_round_dC_to_c(int16_t temperature_dC);

bool tmp102_init(I2C_HandleTypeDef *i2c)
{
    if ((i2c == NULL) || (i2c->Instance != I2C1)) {
        return false;
    }

    memset(&s_tmp102, 0, sizeof(s_tmp102));
    s_tmp102.i2c = i2c;
    s_tmp102.initialized = true;
    s_tmp102.max_temperature_dC = INT16_MIN;
    return true;
}

void tmp102_set_enabled(bool enabled)
{
    if (!s_tmp102.initialized) {
        return;
    }

    if (enabled == s_tmp102.enabled) {
        return;
    }

    s_tmp102.enabled = enabled;

    if (!enabled) {
        HAL_GPIO_WritePin(Temp_PWR_GPIO_Port, Temp_PWR_Pin, GPIO_PIN_RESET);
        s_tmp102.powered = false;
        s_tmp102.configured = false;
        s_tmp102.temperature_valid = false;
        s_tmp102.alert_irq_pending = false;
        return;
    }

    HAL_GPIO_WritePin(Temp_PWR_GPIO_Port, Temp_PWR_Pin, GPIO_PIN_SET);
    s_tmp102.powered = true;
    s_tmp102.configured = false;
    s_tmp102.temperature_valid = false;
    s_tmp102.power_on_tick_ms = HAL_GetTick();
    s_tmp102.last_idle_sample_tick_ms = 0u;
    s_tmp102.next_retry_tick_ms = s_tmp102.power_on_tick_ms;
    s_tmp102.alert_irq_pending = true;
}

void tmp102_task(void)
{
    uint32_t now_tick_ms;
    uint32_t now_seconds;
    rtc_datetime_t now_datetime;
    bool alarm_active_now;

    if (!s_tmp102.initialized || !s_tmp102.enabled || !s_tmp102.powered) {
        return;
    }

    now_tick_ms = HAL_GetTick();

    if ((now_tick_ms - s_tmp102.power_on_tick_ms) < TMP102_POWER_SETTLE_MS) {
        return;
    }

    if (!tmp102_tick_due(now_tick_ms, s_tmp102.next_retry_tick_ms)) {
        return;
    }

    if (!s_tmp102.configured) {
        if (!tmp102_configure_sensor()) {
            s_tmp102.temperature_valid = false;
            s_tmp102.next_retry_tick_ms = now_tick_ms + TMP102_RETRY_PERIOD_MS;
            return;
        }
        s_tmp102.configured = true;
        s_tmp102.alert_irq_pending = true;
    }

    alarm_active_now = tmp102_alarm_pin_is_active();
    if (s_tmp102.alert_irq_pending || (alarm_active_now != s_tmp102.alarm_active)) {
        if (alarm_active_now) {
            tmp102_handle_alarm_assert();
        } else {
            tmp102_handle_alarm_deassert();
        }
        s_tmp102.alert_irq_pending = false;
    }

    if (!s_tmp102.alarm_active) {
        if (!s_tmp102.temperature_valid ||
            ((now_tick_ms - s_tmp102.last_idle_sample_tick_ms) >= TMP102_IDLE_SAMPLE_PERIOD_MS)) {
            int16_t temperature_dC;

            if (tmp102_read_temperature_internal(&temperature_dC)) {
                s_tmp102.temperature_dC = temperature_dC;
                s_tmp102.temperature_valid = true;
                s_tmp102.last_idle_sample_tick_ms = now_tick_ms;
                s_tmp102.next_retry_tick_ms = now_tick_ms;
            } else {
                s_tmp102.temperature_valid = false;
                s_tmp102.next_retry_tick_ms = now_tick_ms + TMP102_RETRY_PERIOD_MS;
            }
        }
        return;
    }

    if (!tmp102_get_now_seconds(&now_seconds, &now_datetime)) {
        return;
    }

    if ((now_seconds - s_tmp102.last_overtemp_sample_s) >= TMP102_OVERTEMP_SAMPLE_PERIOD_S) {
        int16_t temperature_dC;

        if (tmp102_read_temperature_internal(&temperature_dC)) {
            s_tmp102.temperature_dC = temperature_dC;
            s_tmp102.temperature_valid = true;
            s_tmp102.last_overtemp_sample_s = now_seconds;
            s_tmp102.last_idle_sample_tick_ms = now_tick_ms;
            s_tmp102.next_retry_tick_ms = now_tick_ms;
            tmp102_update_max_temperature(temperature_dC, &now_datetime, true);
        } else {
            s_tmp102.temperature_valid = false;
            s_tmp102.next_retry_tick_ms = now_tick_ms + TMP102_RETRY_PERIOD_MS;
        }
    }
}

void tmp102_notify_alert_irq(void)
{
    if (!s_tmp102.initialized) {
        return;
    }

    s_tmp102.alert_irq_pending = true;
}

bool tmp102_read_temperature_now(int16_t *temperature_dC)
{
    uint32_t now_tick_ms;
    int16_t latest_temperature_dC;

    if ((temperature_dC == NULL) ||
        !s_tmp102.initialized ||
        !s_tmp102.enabled ||
        !s_tmp102.powered) {
        return false;
    }

    now_tick_ms = HAL_GetTick();
    if ((now_tick_ms - s_tmp102.power_on_tick_ms) < TMP102_POWER_SETTLE_MS) {
        return false;
    }

    if (!s_tmp102.configured) {
        if (!tmp102_configure_sensor()) {
            s_tmp102.temperature_valid = false;
            s_tmp102.next_retry_tick_ms = now_tick_ms + TMP102_RETRY_PERIOD_MS;
            return false;
        }
        s_tmp102.configured = true;
    }

    if (!tmp102_read_temperature_internal(&latest_temperature_dC)) {
        s_tmp102.temperature_valid = false;
        s_tmp102.next_retry_tick_ms = now_tick_ms + TMP102_RETRY_PERIOD_MS;
        return false;
    }

    s_tmp102.temperature_dC = latest_temperature_dC;
    s_tmp102.temperature_valid = true;
    s_tmp102.last_idle_sample_tick_ms = now_tick_ms;
    s_tmp102.next_retry_tick_ms = now_tick_ms;
    *temperature_dC = latest_temperature_dC;

    if (s_tmp102.alarm_active) {
        uint32_t now_seconds;
        rtc_datetime_t now_datetime;

        if (tmp102_get_now_seconds(&now_seconds, &now_datetime)) {
            s_tmp102.last_overtemp_sample_s = now_seconds;
            tmp102_update_max_temperature(latest_temperature_dC, &now_datetime, true);
        }
    }

    return true;
}

bool tmp102_read_temperature_now_rounded(int8_t *temperature_c)
{
    int16_t temperature_dC;

    if ((temperature_c == NULL) || !tmp102_read_temperature_now(&temperature_dC)) {
        return false;
    }

    *temperature_c = tmp102_round_dC_to_c(temperature_dC);
    return true;
}

bool tmp102_get_last_temperature(int16_t *temperature_dC)
{
    if ((!s_tmp102.temperature_valid) || (temperature_dC == NULL)) {
        return false;
    }

    *temperature_dC = s_tmp102.temperature_dC;
    return true;
}

bool tmp102_get_last_temperature_rounded(int8_t *temperature_c)
{
    int16_t temperature_dC;

    if ((temperature_c == NULL) || !tmp102_get_last_temperature(&temperature_dC)) {
        return false;
    }

    *temperature_c = tmp102_round_dC_to_c(temperature_dC);
    return true;
}

uint32_t tmp102_get_overtemp_time_s(void)
{
    return s_tmp102.overtemp_completed_s + tmp102_get_active_overtemp_seconds();
}

void tmp102_alarm_reset(void)
{
    s_tmp102.overtemp_completed_s = 0u;
    s_tmp102.max_temperature_valid = false;
    s_tmp102.max_temperature_dC = INT16_MIN;
    memset(&s_tmp102.max_temperature_datetime, 0, sizeof(s_tmp102.max_temperature_datetime));
    memset(&s_tmp102.overtemp_start_datetime, 0, sizeof(s_tmp102.overtemp_start_datetime));
    memset(&s_tmp102.last_overtemp_end_datetime, 0, sizeof(s_tmp102.last_overtemp_end_datetime));
    s_tmp102.overtemp_start_valid = false;
    s_tmp102.last_overtemp_end_valid = false;

    if (s_tmp102.alarm_active) {
        uint32_t now_seconds;
        rtc_datetime_t now_datetime;

        if (tmp102_get_now_seconds(&now_seconds, &now_datetime)) {
            s_tmp102.overtemp_start_datetime = now_datetime;
            s_tmp102.overtemp_start_valid = true;
            s_tmp102.last_overtemp_sample_s = now_seconds;
        }
    }
}

void tmp102_export_backup(tmp102_backup_t *backup)
{
    if (backup == NULL) {
        return;
    }

    memset(backup, 0, sizeof(*backup));
    backup->version = TMP102_BACKUP_VERSION;
    backup->overtemp_time_s = s_tmp102.overtemp_completed_s;
    backup->last_temperature_dC = s_tmp102.temperature_valid ? s_tmp102.temperature_dC : INT16_MIN;
    backup->max_temperature_dC = s_tmp102.max_temperature_valid ? s_tmp102.max_temperature_dC : INT16_MIN;

    if (s_tmp102.max_temperature_valid) {
        backup->flags |= TMP102_BACKUP_FLAG_MAX_VALID;
        backup->max_temperature_datetime = s_tmp102.max_temperature_datetime;
    }
    if (s_tmp102.alarm_active) {
        backup->flags |= TMP102_BACKUP_FLAG_OVERTEMP_ACTIVE;
    }
    if (s_tmp102.overtemp_start_valid) {
        backup->flags |= TMP102_BACKUP_FLAG_OVERTEMP_START_VALID;
        backup->overtemp_start_datetime = s_tmp102.overtemp_start_datetime;
    }
    if (s_tmp102.last_overtemp_end_valid) {
        backup->flags |= TMP102_BACKUP_FLAG_LAST_END_VALID;
        backup->last_overtemp_end_datetime = s_tmp102.last_overtemp_end_datetime;
    }
}

bool tmp102_import_backup(const tmp102_backup_t *backup)
{
    if ((backup == NULL) || (backup->version != TMP102_BACKUP_VERSION)) {
        return false;
    }

    s_tmp102.overtemp_completed_s = backup->overtemp_time_s;
    s_tmp102.max_temperature_valid = ((backup->flags & TMP102_BACKUP_FLAG_MAX_VALID) != 0u);
    s_tmp102.alarm_active = ((backup->flags & TMP102_BACKUP_FLAG_OVERTEMP_ACTIVE) != 0u);
    s_tmp102.overtemp_start_valid = ((backup->flags & TMP102_BACKUP_FLAG_OVERTEMP_START_VALID) != 0u);
    s_tmp102.last_overtemp_end_valid = ((backup->flags & TMP102_BACKUP_FLAG_LAST_END_VALID) != 0u);

    if (backup->last_temperature_dC != INT16_MIN) {
        s_tmp102.temperature_dC = backup->last_temperature_dC;
        s_tmp102.temperature_valid = true;
    } else {
        s_tmp102.temperature_valid = false;
    }

    if (s_tmp102.max_temperature_valid) {
        s_tmp102.max_temperature_dC = backup->max_temperature_dC;
        s_tmp102.max_temperature_datetime = backup->max_temperature_datetime;
    } else {
        s_tmp102.max_temperature_dC = INT16_MIN;
        memset(&s_tmp102.max_temperature_datetime, 0, sizeof(s_tmp102.max_temperature_datetime));
    }

    if (s_tmp102.overtemp_start_valid) {
        s_tmp102.overtemp_start_datetime = backup->overtemp_start_datetime;
    } else {
        memset(&s_tmp102.overtemp_start_datetime, 0, sizeof(s_tmp102.overtemp_start_datetime));
    }

    if (s_tmp102.last_overtemp_end_valid) {
        s_tmp102.last_overtemp_end_datetime = backup->last_overtemp_end_datetime;
    } else {
        memset(&s_tmp102.last_overtemp_end_datetime, 0, sizeof(s_tmp102.last_overtemp_end_datetime));
    }

    if (s_tmp102.alarm_active) {
        uint32_t now_seconds;
        rtc_datetime_t now_datetime;

        if (tmp102_get_now_seconds(&now_seconds, &now_datetime)) {
            s_tmp102.last_overtemp_sample_s = now_seconds;
        } else {
            s_tmp102.last_overtemp_sample_s = 0u;
        }
    }

    return true;
}

bool tmp102_get_status(tmp102_status_t *status)
{
    if ((status == NULL) || !s_tmp102.initialized) {
        return false;
    }

    status->powered = s_tmp102.powered;
    status->configured = s_tmp102.configured;
    status->enabled = s_tmp102.enabled;
    status->alarm_active = s_tmp102.alarm_active;
    status->temperature_valid = s_tmp102.temperature_valid;
    status->max_temperature_valid = s_tmp102.max_temperature_valid;
    status->overtemp_start_valid = s_tmp102.overtemp_start_valid;
    status->last_overtemp_end_valid = s_tmp102.last_overtemp_end_valid;
    status->temperature_dC = s_tmp102.temperature_dC;
    status->max_temperature_dC = s_tmp102.max_temperature_dC;
    status->overtemp_time_s = tmp102_get_overtemp_time_s();
    status->max_temperature_datetime = s_tmp102.max_temperature_datetime;
    status->overtemp_start_datetime = s_tmp102.overtemp_start_datetime;
    status->last_overtemp_end_datetime = s_tmp102.last_overtemp_end_datetime;
    return true;
}

static bool tmp102_write_register(uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t buffer[3];

    if ((data == NULL) || (len > 2u)) {
        return false;
    }

    buffer[0] = reg;
    memcpy(&buffer[1], data, len);
    return (HAL_I2C_Master_Transmit(s_tmp102.i2c,
                                    (uint16_t)(TMP102_I2C_ADDR << 1),
                                    buffer,
                                    (uint16_t)(len + 1u),
                                    TMP102_I2C_TIMEOUT_MS) == HAL_OK);
}

static bool tmp102_read_register(uint8_t reg, uint8_t *data, uint16_t len)
{
    if (data == NULL) {
        return false;
    }

    return (HAL_I2C_Mem_Read(s_tmp102.i2c,
                             (uint16_t)(TMP102_I2C_ADDR << 1),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             len,
                             TMP102_I2C_TIMEOUT_MS) == HAL_OK);
}

static int16_t tmp102_raw_to_dC(uint16_t raw)
{
    int16_t sample = (int16_t)raw;
    int32_t value_dC;

    sample >>= 4;
    value_dC = ((int32_t)sample * 10);

    if (value_dC >= 0) {
        value_dC = (value_dC + 8) / 16;
    } else {
        value_dC = (value_dC - 8) / 16;
    }

    return (int16_t)value_dC;
}

static uint16_t tmp102_dC_to_raw(int16_t temperature_dC)
{
    int32_t raw = ((int32_t)temperature_dC * 16);

    if (raw >= 0) {
        raw = (raw + 5) / 10;
    } else {
        raw = (raw - 5) / 10;
    }

    return (uint16_t)(((int16_t)raw) << 4);
}

static bool tmp102_configure_sensor(void)
{
    uint8_t cfg[2];
    uint8_t threshold[2];
    uint16_t raw;

    cfg[0] = TMP102_CONFIG_MSB;
    cfg[1] = TMP102_CONFIG_LSB;
    if (!tmp102_write_register(TMP102_REG_CONFIGURATION, cfg, sizeof(cfg))) {
        return false;
    }

    raw = tmp102_dC_to_raw(TMP102_OVERTEMP_OFF_dC);
    threshold[0] = (uint8_t)(raw >> 8);
    threshold[1] = (uint8_t)(raw & 0xFFu);
    if (!tmp102_write_register(TMP102_REG_TLOW, threshold, sizeof(threshold))) {
        return false;
    }

    raw = tmp102_dC_to_raw(TMP102_OVERTEMP_ON_dC);
    threshold[0] = (uint8_t)(raw >> 8);
    threshold[1] = (uint8_t)(raw & 0xFFu);
    if (!tmp102_write_register(TMP102_REG_THIGH, threshold, sizeof(threshold))) {
        return false;
    }

    return true;
}

static bool tmp102_read_temperature_internal(int16_t *temperature_dC)
{
    uint8_t raw_bytes[2];
    uint16_t raw;

    if ((temperature_dC == NULL) || !tmp102_read_register(TMP102_REG_TEMPERATURE, raw_bytes, sizeof(raw_bytes))) {
        return false;
    }

    raw = (uint16_t)(((uint16_t)raw_bytes[0] << 8) | raw_bytes[1]);
    *temperature_dC = tmp102_raw_to_dC(raw);
    return true;
}

static bool tmp102_alarm_pin_is_active(void)
{
    return (HAL_GPIO_ReadPin(Temp_Alert_GPIO_Port, Temp_Alert_Pin) == GPIO_PIN_RESET);
}

static bool tmp102_datetime_to_seconds(const rtc_datetime_t *datetime, uint32_t *seconds)
{
    static const uint16_t days_before_month[12] = {
        0u, 31u, 59u, 90u, 120u, 151u, 181u, 212u, 243u, 273u, 304u, 334u
    };
    uint32_t days = 0u;
    uint16_t year;

    if ((datetime == NULL) || (seconds == NULL) ||
        (datetime->date.month < 1u) || (datetime->date.month > 12u) ||
        (datetime->date.day < 1u) || (datetime->date.day > 31u)) {
        return false;
    }

    for (year = 2000u; year < datetime->date.year; year++) {
        days += tmp102_is_leap_year(year) ? 366u : 365u;
    }

    days += days_before_month[datetime->date.month - 1u];
    if ((datetime->date.month > 2u) && tmp102_is_leap_year(datetime->date.year)) {
        days += 1u;
    }
    days += (uint32_t)(datetime->date.day - 1u);

    *seconds = (days * 86400u) +
               ((uint32_t)datetime->time.hours * 3600u) +
               ((uint32_t)datetime->time.minutes * 60u) +
               (uint32_t)datetime->time.seconds;
    return true;
}

static bool tmp102_is_leap_year(uint16_t year)
{
    return (((year % 4u) == 0u) && (((year % 100u) != 0u) || ((year % 400u) == 0u)));
}

static uint32_t tmp102_get_active_overtemp_seconds(void)
{
    rtc_datetime_t now;
    uint32_t start_seconds;
    uint32_t now_seconds;

    if (!s_tmp102.alarm_active || !s_tmp102.overtemp_start_valid) {
        return 0u;
    }

    if (!rtc_driver_get_datetime(&now)) {
        return 0u;
    }

    if (!tmp102_datetime_to_seconds(&s_tmp102.overtemp_start_datetime, &start_seconds) ||
        !tmp102_datetime_to_seconds(&now, &now_seconds) ||
        (now_seconds < start_seconds)) {
        return 0u;
    }

    return now_seconds - start_seconds;
}

static bool tmp102_get_now_seconds(uint32_t *seconds, rtc_datetime_t *datetime)
{
    rtc_datetime_t current;

    if (!rtc_driver_get_datetime(&current) || !tmp102_datetime_to_seconds(&current, seconds)) {
        return false;
    }

    if (datetime != NULL) {
        *datetime = current;
    }

    return true;
}

static void tmp102_update_max_temperature(int16_t temperature_dC, const rtc_datetime_t *datetime, bool datetime_valid)
{
    if (!s_tmp102.max_temperature_valid || (temperature_dC > s_tmp102.max_temperature_dC)) {
        s_tmp102.max_temperature_valid = true;
        s_tmp102.max_temperature_dC = temperature_dC;
        if (datetime_valid && (datetime != NULL)) {
            s_tmp102.max_temperature_datetime = *datetime;
        } else {
            memset(&s_tmp102.max_temperature_datetime, 0, sizeof(s_tmp102.max_temperature_datetime));
        }
    }
}

static void tmp102_handle_alarm_assert(void)
{
    rtc_datetime_t now_datetime;
    uint32_t now_seconds;
    int16_t temperature_dC;

    s_tmp102.alarm_active = true;
    if (tmp102_get_now_seconds(&now_seconds, &now_datetime)) {
        s_tmp102.overtemp_start_datetime = now_datetime;
        s_tmp102.overtemp_start_valid = true;
        s_tmp102.last_overtemp_sample_s = now_seconds;
    } else {
        s_tmp102.overtemp_start_valid = false;
        s_tmp102.last_overtemp_sample_s = 0u;
    }

    if (tmp102_read_temperature_internal(&temperature_dC)) {
        s_tmp102.temperature_dC = temperature_dC;
        s_tmp102.temperature_valid = true;
        s_tmp102.last_idle_sample_tick_ms = HAL_GetTick();
        tmp102_update_max_temperature(temperature_dC, &now_datetime, s_tmp102.overtemp_start_valid);
    } else {
        s_tmp102.temperature_valid = false;
        s_tmp102.next_retry_tick_ms = HAL_GetTick() + TMP102_RETRY_PERIOD_MS;
    }
}

static void tmp102_handle_alarm_deassert(void)
{
    rtc_datetime_t now_datetime;
    uint32_t start_seconds;
    uint32_t end_seconds;

    if (s_tmp102.overtemp_start_valid &&
        rtc_driver_get_datetime(&now_datetime) &&
        tmp102_datetime_to_seconds(&s_tmp102.overtemp_start_datetime, &start_seconds) &&
        tmp102_datetime_to_seconds(&now_datetime, &end_seconds) &&
        (end_seconds >= start_seconds)) {
        s_tmp102.overtemp_completed_s += (end_seconds - start_seconds);
        s_tmp102.last_overtemp_end_datetime = now_datetime;
        s_tmp102.last_overtemp_end_valid = true;
    }

    s_tmp102.alarm_active = false;
    s_tmp102.overtemp_start_valid = false;
}

static bool tmp102_tick_due(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0);
}

static int8_t tmp102_round_dC_to_c(int16_t temperature_dC)
{
    int16_t rounded;

    if (temperature_dC >= 0) {
        rounded = (temperature_dC + 5) / 10;
    } else {
        rounded = (temperature_dC - 5) / 10;
    }

    if (rounded > INT8_MAX) {
        rounded = INT8_MAX;
    }
    if (rounded < INT8_MIN) {
        rounded = INT8_MIN;
    }

    return (int8_t)rounded;
}
