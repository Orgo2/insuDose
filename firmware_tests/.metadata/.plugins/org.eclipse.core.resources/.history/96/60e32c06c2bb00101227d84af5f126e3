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

static uint8_t s_inited = 0;
static bool    s_enabled = false; // čo sme posledne nastavili na CEN

static void charger_gpio_init_once(void)
{
    if (s_inited) return;
    s_inited = 1;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // CEN: výstup, bez pull (STNS01 má interný ~500k pull-up na LDO)
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = CHARGER_CEN_Pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CHARGER_CEN_GPIO_Port, &gi);

    // CHG: vstup, bez pull (máš externý 470k pull-up)
    gi.Pin  = CHARGER_CHG_Pin;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CHARGER_CHG_GPIO_Port, &gi);

    // BAT ADC: analóg (bez pull)
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
}

void charger_init(bool enable_on_boot)
{
    charger_gpio_init_once();
    // Bezpečný default s nenabíjateľnou batériou: vypnúť
    charger_set_enabled(enable_on_boot ? true : false);
}

static inline uint8_t chg_is_low(void)
{
#if CHARGER_CHG_ACTIVE_LOW
    return (HAL_GPIO_ReadPin(CHARGER_CHG_GPIO_Port, CHARGER_CHG_Pin) == GPIO_PIN_RESET);
#else
    return (HAL_GPIO_ReadPin(CHARGER_CHG_GPIO_Port, CHARGER_CHG_Pin) == GPIO_PIN_SET);
#endif
}

int charger_get_status_fast(bool vin_present)
{
    if (chg_is_low()) {
        return CHARGER_STATUS_CHARGING; // 1
    }

    // CHG=HIGH (high-Z cez pull-up) → môže znamenať "full" ALEBO "nenabíja"
    // Rozlíšime cez VIN + povolenie CEN:
    if (vin_present && s_enabled) {
        return CHARGER_STATUS_FULL;     // 2 (standby/termination)
    }
    return CHARGER_STATUS_IDLE;         // 0 (VIN off alebo CEN off)
}

int charger_get_status_blocking(bool vin_present, uint32_t observe_ms)
{
    // Pozorovanie togglingu ~1 Hz na CHG pre detekciu faultu
    // (datasheet: CHG toggling pri fault, typicky ~1 Hz).
    observe_ms = 1200;
    const uint32_t step_ms = 100;
    uint8_t last = chg_is_low();
    uint8_t toggles = 0;
    uint32_t waited = 0;

    while (waited < observe_ms) {
        HAL_Delay(step_ms);
        waited += step_ms;
        uint8_t now = chg_is_low();
        if (now != last) {
            toggles++;
            last = now;
            if (toggles >= 2) { // stačia 2 zmeny, aby to bol spoľahlivo "fault"
                return CHARGER_STATUS_FAULT; // 3
            }
        }
    }

    // Bez togglingu → použijeme rýchlu logiku
    return charger_get_status_fast(vin_present);
}

bool charger_is_charging_fast(void)
{
    return (charger_get_status_fast(true) == CHARGER_STATUS_CHARGING);
}

int32_t charger_read_bat(void)
{
#if CHARGER_USE_ADC && defined(CHARGER_ADC_CHANNEL)
    // 1) Zmeraj VREFINT (interný kanál) – získame ACTUAL VDDA v mV
    ADC_ChannelConfTypeDef s = {0};
    s.Channel      = ADC_CHANNEL_VREFINT;
    s.Rank         = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;   // dlhé vzorkovanie kvôli impedancii
    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) return -1;
    if (HAL_ADC_Start(&hadc1) != HAL_OK) return -1;
    if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) { HAL_ADC_Stop(&hadc1); return -1; }
    uint16_t raw_vref = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    uint32_t vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw_vref, ADC_RESOLUTION_12B); // mV

    // 2) Zmeraj VBAT (tvoj PA1 kanál) ziskame napatie na baterii
    s.Channel      = CHARGER_ADC_CHANNEL;          // napr. ADC_CHANNEL_6 (doplníš v .h keď bude .ioc)
    s.Rank         = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;   // vysoká impedancia deliča 470k//47k
    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) return -1;
    if (HAL_ADC_Start(&hadc1) != HAL_OK) return -1;
    if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) { HAL_ADC_Stop(&hadc1); return -1; }
    uint16_t raw_vbat = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    // 3) Prepočet: raw -> mV na ADC pine -> mV na batérii (delič 470k/47k)
    uint32_t mv_adc = ( (uint32_t)raw_vbat * vdda_mv ) / 4095u;
    uint32_t mv_bat = ( mv_adc * (uint32_t)(CHARGER_RTOP_OHM + CHARGER_RBOT_OHM) ) / (uint32_t)CHARGER_RBOT_OHM;
    return (int32_t)mv_bat;
#else
    return -1;
#endif
}
