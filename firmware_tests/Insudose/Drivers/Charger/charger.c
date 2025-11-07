/*
 * charger.c
 *
 *  Created on: Aug 14, 2025
 *      Author: Orgo
 * HAL-only implementacia pre STNS01 + low-battery ADC watchdog.
 *
 * Klucove body:
 *  - ADC sa spusta len pri jednorazovom merani VBAT a pri aktivacci watchdogu(slabnuca batt).
 *  - Hysteréza je realizovana prepinanim AWD okna:
 *      NORMAL: <Sleep_cnt .. 4095>  -> triger pri poklese pod Sleep (LOW)
 *      LOW:    <0 .. Recover_cnt>   -> triger pri vzostupe nad Recover (HIGH)
 *  - IRQ callback len preklapa flag a okno; ostatne moduly sleduju flag.
 */

#include "charger.h"
#include "stm32wbxx_hal.h"

#if CHARGER_USE_ADC
extern ADC_HandleTypeDef hadc1;
#endif

/* ======= Charger GPIO (CEN/CHG) ======= */

static inline uint8_t chg_pin_is_low(void)
{
#if CHARGER_CHG_ACTIVE_LOW
    return (HAL_GPIO_ReadPin(CHARGER_CHG_GPIO_Port, CHARGER_CHG_Pin) == GPIO_PIN_RESET);
#else
    return (HAL_GPIO_ReadPin(CHARGER_CHG_GPIO_Port, CHARGER_CHG_Pin) == GPIO_PIN_SET);
#endif
}

static inline void cen_write(bool enable)
{
#if CHARGER_CEN_ACTIVE_HIGH
    HAL_GPIO_WritePin(CHARGER_CEN_GPIO_Port, CHARGER_CEN_Pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(CHARGER_CEN_GPIO_Port, CHARGER_CEN_Pin,
                      enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

void charger_set_enabled(bool enable)
{
    cen_write(enable);
}

/* ======= FAST status a VBAT meranie ======= */

#if CHARGER_USE_ADC
/* Jednorazovo zmeraj VDDA cez VREFINT (presny prepočet mV) */
static uint32_t read_vdda_mv_once(void)
{
    ADC_ChannelConfTypeDef s = {0};
    s.Channel      = ADC_CHANNEL_VREFINT;
    s.Rank         = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) return 3000;
    if (HAL_ADC_Start(&hadc1) != HAL_OK)            return 3000;
    if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) { HAL_ADC_Stop(&hadc1); return 3000; }
    uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return __HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw, ADC_RESOLUTION_12B);
}

/* Jednorazovo zmeraj VBAT v mV cez delič */
static int32_t read_vbat_mv_once(uint32_t vdda_mv)
{
    ADC_ChannelConfTypeDef s = {0};
    s.Channel      = CHARGER_ADC_CHANNEL;
    s.Rank         = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) return -1;
    if (HAL_ADC_Start(&hadc1) != HAL_OK)            return -1;
    if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) { HAL_ADC_Stop(&hadc1); return -1; }
    uint16_t raw_vbat = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    /* Prepočet raw -> mV na ADC pine -> mV na batérii */
    uint32_t mv_adc = ((uint32_t)raw_vbat * vdda_mv) / 4095u;
    uint32_t mv_bat = (mv_adc * (uint32_t)(CHARGER_RTOP_OHM + CHARGER_RBOT_OHM)) / (uint32_t)CHARGER_RBOT_OHM;
    return (int32_t)mv_bat;
}
#endif /* CHARGER_USE_ADC */

CHARGER_Fast charger_get_fast(void)
{
    CHARGER_Fast out;
    out.status = chg_pin_is_low() ? CHARGER_STATUS_CHARGING : CHARGER_STATUS_IDLE;

#if CHARGER_USE_ADC
    uint32_t vdda = read_vdda_mv_once();
    out.batt_mv   = read_vbat_mv_once(vdda);
    /* Po každom meraní sa nižšie vyhodnotí (dis)arm watchdog podľa politík. */
#else
    out.batt_mv   = -1;
#endif

    /* Vyhodnotenie politiky + (dis)arm watchdog sa rieši v helperi nižšie. */
    /* Ak ADC nepouzivas, helper ticho nerobi nic. */
    extern void charger__policy_eval_and_arm(int32_t last_batt_mv, uint32_t last_vdda_mv);
#if CHARGER_USE_ADC
    charger__policy_eval_and_arm(out.batt_mv, vdda);
#else
    charger__policy_eval_and_arm(-1, 0);
#endif

    return out;
}

/* ======= Blocking FAULT detekcia (1200 ms fixne) ======= */

CHARGER_Status charger_get_status_blocking(void)
{
    const uint32_t observe_ms = 1200;  // fixne, podla tvojej poziadavky
    const uint32_t step_ms    = 100;

    uint8_t last    = chg_pin_is_low();
    uint8_t toggles = 0;
    uint32_t waited = 0;

    while (waited < observe_ms) {
        HAL_Delay(step_ms);
        waited += step_ms;

        uint8_t now = chg_pin_is_low();
        if (now != last) {
            toggles++;
            last = now;
            if (toggles >= 2) {
                return CHARGER_STATUS_FAULT;
            }
        }
    }

    return chg_pin_is_low() ? CHARGER_STATUS_CHARGING : CHARGER_STATUS_IDLE;
}

/* ======= Low-battery politika + ADC watchdog (HAL-only) ======= */

#if CHARGER_USE_ADC
/* Politika (v mV), nastavuje pouzivatel cez charger_batt_watchdog_config() */
static uint32_t s_sleep_mv   = 3300;  /* uloz sa na spánok */
static uint32_t s_hyst_mv    = 200;   /* prebud sa na sleep + hyst */
static uint32_t s_bwdg_mv    = 100;   /* zapni AWD, ak VBAT <= sleep + bwdg */

/* Stav watchdogu a prahy v „counts“ */
static volatile bool s_batt_low = false;

typedef enum { MON_DISABLED = 0, MON_NORMAL, MON_LOW } mon_state_t;
static volatile mon_state_t s_mon_state = MON_DISABLED;

static uint16_t s_sleep_cnt   = 0;  /* ADC threshold pre pokles */
static uint16_t s_recover_cnt = 0;  /* ADC threshold pre navrat */

/* Prepočet VBAT mV -> ADC counts, s ohľadom na delič a VDDA */
static uint16_t mv_to_counts(uint32_t mv_vbat, uint32_t vdda_mv)
{
    uint32_t vadc_mv = (mv_vbat * (uint32_t)CHARGER_RBOT_OHM) /
                       (uint32_t)(CHARGER_RTOP_OHM + CHARGER_RBOT_OHM);
    if (vdda_mv == 0) vdda_mv = 3000;
    uint32_t counts = (vadc_mv * 4095u) / vdda_mv;
    if (counts > 4095u) counts = 4095u;
    return (uint16_t)counts;
}

/* Nastavenie AWD okna (HAL) podle stavu */
static void awd_apply_window_normal(void)
{
    ADC_AnalogWDGConfTypeDef awd = {0};
    awd.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
    awd.WatchdogMode   = ADC_ANALOGWATCHDOG_SINGLE_REG;   /* len regular kanál */
    awd.Channel        = CHARGER_ADC_CHANNEL;
    awd.ITMode         = ENABLE;                           /* IRQ povolene */
    awd.HighThreshold  = 4095;                             /* horný strop */
    awd.LowThreshold   = s_sleep_cnt;                      /* spodná hranica = Sleep */

    (void)HAL_ADC_AnalogWDGConfig(&hadc1, &awd);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_AWD1);
}

static void awd_apply_window_recover(void)
{
    ADC_AnalogWDGConfTypeDef awd = {0};
    awd.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
    awd.WatchdogMode   = ADC_ANALOGWATCHDOG_SINGLE_REG;
    awd.Channel        = CHARGER_ADC_CHANNEL;
    awd.ITMode         = ENABLE;
    awd.HighThreshold  = s_recover_cnt;                    /* horná hranica = Recover */
    awd.LowThreshold   = 0;

    (void)HAL_ADC_AnalogWDGConfig(&hadc1, &awd);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_AWD1);
}

/* Interné: spusti ADC v režime s IT (spotreba len ked bezi);
 * Predpoklad: hadc1.Init.ContinuousConvMode = ENABLE v CubeMX _alebo_ už pri tvojom ADC init-e.
 * (Continuous režim NEzvyšuje spotrebu, kym ADC nie je spustene; zacne az po HAL_ADC_Start_IT.) */
static void adc_start_for_watchdog(void)
{
    /* Uisti sa, ze merame nas VBAT kanal (regular group) */
    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = CHARGER_ADC_CHANNEL;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    (void)HAL_ADC_ConfigChannel(&hadc1, &ch);

    (void)HAL_ADC_Start_IT(&hadc1);
}

static void adc_stop_watchdog(void)
{
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_AWD1);
    (void)HAL_ADC_Stop_IT(&hadc1);
}

/* ===== API: konfigurácia politiky ===== */

void charger_batt_watchdog_config(uint32_t sleep_mv, uint32_t hyst_mv, uint32_t bwdg_mv)
{
    s_sleep_mv = sleep_mv;
    s_hyst_mv  = hyst_mv;
    s_bwdg_mv  = bwdg_mv;

    /* Disarm pri zmene politiky (bezpecne default) */
    charger_batt_watchdog_disarm();
}

/* ===== API: flag + disarm ===== */

bool charger_batt_low_flag(void)
{
    return s_batt_low;
}

void charger_batt_watchdog_disarm(void)
{
    adc_stop_watchdog();
    s_batt_low  = false;
    s_mon_state = MON_DISABLED;
    s_sleep_cnt = 0;
    s_recover_cnt = 0;
}

/* ===== Interné: vyhodnotenie po každom meraní a (dis)arm watchdog =====
 * Volané z charger_get_fast() po jednorazovom VBAT meraní.
 *
 * Logika:
 *  - Ak VBAT <= Sleep_V       -> nastav low flag a armuj okno "recover" (LOW state).
 *  - Inak ak VBAT <= Sleep_V + Bwdg_V -> armuj okno "normal" (NORMAL state).
 *  - Inak -> disarm (šetri spotrebu).
 */
void charger__policy_eval_and_arm(int32_t last_batt_mv, uint32_t last_vdda_mv)
{
    (void)last_vdda_mv; /* VDDA sme uz pouzili pri merani; prahy pocitame nizsie samostatne */

    if (last_batt_mv < 0) {
        /* Bez ADC merania netusime; radsej disarm pre istotu. */
        charger_batt_watchdog_disarm();
        return;
    }

    /* Prepočet prahov na counts podľa aktuálnej (čerstvo zmeranej) VDDA */
    uint32_t vdda_mv = read_vdda_mv_once();
    s_sleep_cnt   = mv_to_counts(s_sleep_mv, vdda_mv);
    s_recover_cnt = mv_to_counts(s_sleep_mv + s_hyst_mv, vdda_mv);
    if (s_recover_cnt <= s_sleep_cnt) s_recover_cnt = s_sleep_cnt + 1;

    if ((uint32_t)last_batt_mv <= s_sleep_mv) {
        /* Uz sme pod prahom -> nastav flag a prepnime AWD na „recover“,
         * aby sme vedeli zaregistrovat zvysenie napätia (prebudenie). */
        s_batt_low  = true;
        s_mon_state = MON_LOW;
        adc_start_for_watchdog();
        awd_apply_window_recover();
        __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_AWD1);
        return;
    }

    if ((uint32_t)last_batt_mv <= (s_sleep_mv + s_bwdg_mv)) {
        /* Sme v okoli prahu -> armuj watchdog na pokles */
        s_batt_low  = false;
        s_mon_state = MON_NORMAL;
        adc_start_for_watchdog();
        awd_apply_window_normal();
        __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_AWD1);
        return;
    }

    /* Daleko od prahu -> vypni watchdog, setri spotrebu */
    charger_batt_watchdog_disarm();
}

/* ===== IRQ callback: HAL zavola pri AWD evente =====
 * NORMAL: triger pri poklese pod s_sleep_cnt -> LOW stav + okno „recover“
 * LOW:    triger pri vzostupe nad s_recover_cnt -> NORMAL stav + okno „normal“
 */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) return;

    uint32_t val = HAL_ADC_GetValue(&hadc1); /* posledna hodnota */

    if (s_mon_state == MON_NORMAL) {
        if (val < s_sleep_cnt) {
            s_batt_low  = true;
            s_mon_state = MON_LOW;
            awd_apply_window_recover(); /* sleduj navrat nad Recover */
        }
    } else if (s_mon_state == MON_LOW) {
        if (val > s_recover_cnt) {
            s_batt_low  = false;
            s_mon_state = MON_NORMAL;
            awd_apply_window_normal(); /* sleduj pokles pod Sleep */
        }
    } else {
        /* MON_DISABLED -> nic */
    }
}

#else  /* !CHARGER_USE_ADC  (ADC vypnuty v projekte) */

void charger_batt_watchdog_config(uint32_t sleep_mv, uint32_t hyst_mv, uint32_t bwdg_mv)
{
    (void)sleep_mv; (void)hyst_mv; (void)bwdg_mv;
}
bool charger_batt_low_flag(void) { return false; }
void charger_batt_watchdog_disarm(void) {}

void charger__policy_eval_and_arm(int32_t last_batt_mv, uint32_t last_vdda_mv)
{
    (void)last_batt_mv; (void)last_vdda_mv;
    /* Bez ADC nerobime nic. */
}

#endif /* CHARGER_USE_ADC */
