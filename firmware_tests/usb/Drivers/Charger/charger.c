/*
 * charger.c
 *
 *  Created on: Aug 14, 2025
 *      Author: Orgo
 */

#include "charger.h"
#include "stm32wbxx_hal.h"

#if CHARGER_USE_ADC
extern ADC_HandleTypeDef hadc1;
#endif

static uint8_t        s_inited = 0;
static bool           s_enabled = false;
static bool           s_charge_allowed = (CHARGER_BATTERY_RECHARGEABLE != 0);
static bool           s_last_vin_present = false;
static uint32_t       s_force_enable_until_ms = 0;
static uint32_t       s_next_measure_ms = 0;
static charger_info_t s_info = {
    .battery_mv     = -1,
    .battery_v      = -1.0f,
    .battery_valid  = false,
    .vin_present    = false,
    .charge_allowed = (CHARGER_BATTERY_RECHARGEABLE != 0),
    .charge_enabled = false,
    .is_charging    = false,
    .battery_low    = false,
    .battery_empty  = false,
    .battery_full   = false,
    .restore_allowed = false,
    .status         = CHARGER_STATUS_IDLE
};

static void charger_gpio_init_once(void);
static void charger_refresh_battery_state(bool force_measure);
static void charger_refresh_runtime_state(bool vin_present);
static bool charger_should_enable_now(bool vin_present);
static bool charger_restore_allowed_now(bool vin_present);

static inline bool tick_reached(uint32_t now, uint32_t target)
{
    return ((int32_t)(now - target) >= 0);
}

static void charger_gpio_init_once(void)
{
    if (s_inited) {
        return;
    }
    s_inited = 1;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // CEN: vystup bez pull.
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = CHARGER_CEN_Pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CHARGER_CEN_GPIO_Port, &gi);

    // CHG: vstup bez pull, na doske je externy pull-up.
    gi.Pin  = CHARGER_CHG_Pin;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CHARGER_CHG_GPIO_Port, &gi);

    // BAT ADC: analog vstup.
    gi.Pin  = CHARGER_BAT_ADC_Pin;
    gi.Mode = GPIO_MODE_ANALOG;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CHARGER_BAT_ADC_Port, &gi);
}

void charger_set_enabled(bool enable)
{
#if CHARGER_CEN_ACTIVE_HIGH
    HAL_GPIO_WritePin(CHARGER_CEN_GPIO_Port, CHARGER_CEN_Pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(CHARGER_CEN_GPIO_Port, CHARGER_CEN_Pin,
                      enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
    s_enabled = enable;
    s_info.charge_enabled = enable;
}

void charger_init(void)
{
    charger_gpio_init_once();

    s_last_vin_present = false;
    s_force_enable_until_ms = HAL_GetTick() + CHARGER_BOOT_FORCE_ON_MS;
    s_next_measure_ms = 0;

    // STNS01 potrebuje po starte CEN=1.
    charger_set_enabled(true);
    charger_refresh_battery_state(true);
    charger_refresh_runtime_state(false);
}

void charger_set_charge_allowed(bool allow)
{
#if CHARGER_BATTERY_RECHARGEABLE
    s_charge_allowed = allow;
#else
    (void)allow;
    s_charge_allowed = false;
#endif

    s_info.charge_allowed = s_charge_allowed;
}

bool charger_get_charge_allowed(void)
{
    return s_charge_allowed;
}

static inline uint8_t chg_is_low(void)
{
#if CHARGER_CHG_ACTIVE_LOW
    return (HAL_GPIO_ReadPin(CHARGER_CHG_GPIO_Port, CHARGER_CHG_Pin) == GPIO_PIN_RESET);
#else
    return (HAL_GPIO_ReadPin(CHARGER_CHG_GPIO_Port, CHARGER_CHG_Pin) == GPIO_PIN_SET);
#endif
}

static void charger_refresh_runtime_state(bool vin_present)
{
    s_info.vin_present    = vin_present;
    s_info.charge_allowed = s_charge_allowed;
    s_info.charge_enabled = s_enabled;
    s_info.status         = charger_get_status_fast(vin_present);
    s_info.is_charging    = (s_info.status == CHARGER_STATUS_CHARGING);
    s_info.restore_allowed = charger_restore_allowed_now(vin_present);
}

static void charger_refresh_battery_state(bool force_measure)
{
    uint32_t now = HAL_GetTick();

    if (!force_measure && s_info.battery_valid && !tick_reached(now, s_next_measure_ms)) {
        return;
    }

    int32_t mv = charger_read_bat();
    s_next_measure_ms = now + CHARGER_MEASURE_PERIOD_MS;

    if (mv < 0) {
        s_info.battery_mv    = -1;
        s_info.battery_v     = -1.0f;
        s_info.battery_valid = false;
        s_info.battery_low   = false;
        s_info.battery_empty = false;
        s_info.battery_full  = false;
        return;
    }

    s_info.battery_mv    = mv;
    s_info.battery_v     = (float)mv / 1000.0f;
    s_info.battery_valid = true;
    s_info.battery_low   = ((uint32_t)mv <= CHARGER_BAT_LOW_MV);
    s_info.battery_empty = ((uint32_t)mv <= CHARGER_BAT_EMPTY_MV);
    s_info.battery_full  = ((uint32_t)mv >= CHARGER_BAT_CHARGE_STOP_MV);
}

static bool charger_should_enable_now(bool vin_present)
{
    if (!vin_present) {
        return false;
    }

#if !CHARGER_BATTERY_RECHARGEABLE
    return false;
#else
    if (!s_charge_allowed) {
        return false;
    }

    // Ked meranie zlyha, nechame nabijanie radsej zapnute.
    if (!s_info.battery_valid) {
        return true;
    }

    // Hysteresis medzi START a STOP limitom.
    if (s_enabled) {
        return ((uint32_t)s_info.battery_mv < CHARGER_BAT_CHARGE_STOP_MV);
    }

    return ((uint32_t)s_info.battery_mv <= CHARGER_BAT_CHARGE_START_MV);
#endif
}

static bool charger_restore_allowed_now(bool vin_present)
{
    if (vin_present) {
        return true;
    }

    return (s_info.battery_valid && ((uint32_t)s_info.battery_mv >= CHARGER_BAT_RESTORE_MV));
}

void charger_task(bool vin_present)
{
    uint32_t now = HAL_GetTick();
    bool vin_rising = (vin_present && !s_last_vin_present);

    // Po prichode VIN drzi CEN chvilu v log.1, aby STNS01 spravne presiel
    // do backup rezimu na bateriu.
    if (vin_rising) {
        s_force_enable_until_ms = now + CHARGER_BOOT_FORCE_ON_MS;
        charger_set_enabled(true);
    }

    charger_refresh_battery_state(vin_rising);

    if (!tick_reached(now, s_force_enable_until_ms)) {
        charger_set_enabled(true);
    } else {
        charger_set_enabled(charger_should_enable_now(vin_present));
    }

    charger_refresh_runtime_state(vin_present);
    s_last_vin_present = vin_present;
}

void charger_get_info(charger_info_t *info)
{
    if (info == NULL) {
        return;
    }

    *info = s_info;
}

float charger_get_battery_voltage(void)
{
    return s_info.battery_v;
}

bool charger_is_battery_low(void)
{
    return s_info.battery_low;
}

bool charger_is_battery_empty(void)
{
    return s_info.battery_empty;
}

bool charger_should_sleep(void)
{
    return (!s_info.vin_present && s_info.battery_valid && s_info.battery_empty);
}

bool charger_should_restore(void)
{
    return charger_restore_allowed_now(s_info.vin_present);
}

void charger_force_measure(void)
{
    s_next_measure_ms = 0;
}

int charger_get_status_fast(bool vin_present)
{
    if (chg_is_low()) {
        return CHARGER_STATUS_CHARGING;
    }

    if (vin_present && s_enabled) {
        return CHARGER_STATUS_FULL;
    }

    return CHARGER_STATUS_IDLE;
}

int charger_get_status_blocking(bool vin_present, uint32_t observe_ms)
{
    const uint32_t step_ms = 100;
    uint8_t last = chg_is_low();
    uint8_t toggles = 0;
    uint32_t waited = 0;

    if (observe_ms < 200u) {
        observe_ms = 1200u;
    }

    while (waited < observe_ms) {
        HAL_Delay(step_ms);
        waited += step_ms;

        uint8_t now = chg_is_low();
        if (now != last) {
            toggles++;
            last = now;

            if (toggles >= 2u) {
                return CHARGER_STATUS_FAULT;
            }
        }
    }

    return charger_get_status_fast(vin_present);
}

bool charger_is_charging_fast(void)
{
    return (charger_get_status_fast(s_info.vin_present) == CHARGER_STATUS_CHARGING);
}

int32_t charger_read_bat(void)
{
#if CHARGER_USE_ADC && defined(CHARGER_ADC_CHANNEL)
    ADC_ChannelConfTypeDef s = {0};
    uint32_t vdda_mv = CHARGER_VREF_MV;

#if CHARGER_USE_VREFINT
    // 1) Zmeraj VREFINT a ziskaj realne VDDA.
    s.Channel      = ADC_CHANNEL_VREFINT;
    s.Rank         = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    s.SingleDiff   = ADC_SINGLE_ENDED;
    s.OffsetNumber = ADC_OFFSET_NONE;
    s.Offset       = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) {
        return -1;
    }
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return -1;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return -1;
    }

    uint16_t raw_vref = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw_vref, ADC_RESOLUTION_12B);
#endif

    // 2) Zmeraj napatie na baterii cez externy delic.
    s.Channel      = CHARGER_ADC_CHANNEL;
    s.Rank         = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    s.SingleDiff   = ADC_SINGLE_ENDED;
    s.OffsetNumber = ADC_OFFSET_NONE;
    s.Offset       = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) {
        return -1;
    }
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return -1;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return -1;
    }

    uint16_t raw_vbat = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    // 3) Prepocet z ADC na realne napatie baterie.
    uint32_t mv_adc = ((uint32_t)raw_vbat * vdda_mv) / 4095u;
    uint32_t mv_bat = (mv_adc * (uint32_t)(CHARGER_RTOP_OHM + CHARGER_RBOT_OHM)) /
                      (uint32_t)CHARGER_RBOT_OHM;

    return (int32_t)mv_bat;
#else
    return -1;
#endif
}
