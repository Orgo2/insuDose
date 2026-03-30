/*
 * charger.h
 *
 *  Created on: Aug 14, 2025
 *      Author: Orgo
 */

#ifndef CHARGER_CHARGER_H_
#define CHARGER_CHARGER_H_
#ifndef CHARGER_H
#define CHARGER_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#pragma once

// -------- PINY GPIO NA STNS01 --------
#define CHARGER_CEN_GPIO_Port   GPIOB
#define CHARGER_CEN_Pin         GPIO_PIN_0   // PB0 -> CEN (active-HIGH)
#define CHARGER_CHG_GPIO_Port   GPIOA
#define CHARGER_CHG_Pin         GPIO_PIN_10  // PA10 -> CHG (active-LOW)
#define CHARGER_BAT_ADC_Port    GPIOA
#define CHARGER_BAT_ADC_Pin     GPIO_PIN_1   // PA1  -> VBAT divider

// -------- LOGIKA STNS01 --------
// CEN: active-HIGH (LOW = disable)
#define CHARGER_CEN_ACTIVE_HIGH 1
// CHG: active-LOW, open-drain; toggling pri fault
#define CHARGER_CHG_ACTIVE_LOW  1

// -------- STATUS KODY --------
#define CHARGER_STATUS_IDLE      0
#define CHARGER_STATUS_CHARGING  1
#define CHARGER_STATUS_FULL      2
#define CHARGER_STATUS_FAULT     3

// -------- KONFIGURACIA BATERIE --------
// LiR2032 je nabijacia, ale ostava to nastavitelne v hlavicke.
#ifndef CHARGER_BATTERY_RECHARGEABLE
#define CHARGER_BATTERY_RECHARGEABLE 1
#endif

// Po boote a po prichode VIN drzi driver CEN chvilu v log.1.
#ifndef CHARGER_BOOT_FORCE_ON_MS
#define CHARGER_BOOT_FORCE_ON_MS 100u
#endif

// Ako casto sa obnovuje meranie baterie a stav chargeru.
#ifndef CHARGER_MEASURE_PERIOD_MS
#define CHARGER_MEASURE_PERIOD_MS 1000u
#endif

// Hysteresis nabijania:
// - ked je CEN vypnute a bateria klesne pod START, nabijanie sa povoli
// - ked je CEN zapnute a bateria dosiahne STOP, nabijanie sa vypne
#ifndef CHARGER_BAT_CHARGE_START_MV
#define CHARGER_BAT_CHARGE_START_MV 4000u
#endif

#ifndef CHARGER_BAT_CHARGE_STOP_MV
#define CHARGER_BAT_CHARGE_STOP_MV 4100u
#endif

// LOW = stav pre upozornenie na slabu bateriu
#ifndef CHARGER_BAT_LOW_MV
#define CHARGER_BAT_LOW_MV 3000u
#endif

// EMPTY = kriticky stav, zariadenie sa ma ulozit a uspat
#ifndef CHARGER_BAT_EMPTY_MV
#define CHARGER_BAT_EMPTY_MV 2800u
#endif

// RESTORE = minimalne napatie, od ktoreho sa smie snapshot obnovit a
// zariadenie moze pokracovat z RAM disku bez externeho napajania.
#ifndef CHARGER_BAT_RESTORE_MV
#define CHARGER_BAT_RESTORE_MV 3200u
#endif

#if (CHARGER_BAT_CHARGE_START_MV >= CHARGER_BAT_CHARGE_STOP_MV)
#error "CHARGER_BAT_CHARGE_START_MV must be lower than CHARGER_BAT_CHARGE_STOP_MV"
#endif

#if (CHARGER_BAT_EMPTY_MV >= CHARGER_BAT_LOW_MV)
#error "CHARGER_BAT_EMPTY_MV must be lower than CHARGER_BAT_LOW_MV"
#endif

#if (CHARGER_BAT_RESTORE_MV <= CHARGER_BAT_EMPTY_MV)
#error "CHARGER_BAT_RESTORE_MV must be higher than CHARGER_BAT_EMPTY_MV"
#endif

// -------- ADC --------
#ifndef CHARGER_USE_ADC
#define CHARGER_USE_ADC 1
#endif

// Ak je 1, VDDA sa pocita z vnutorneho VREFINT.
// Ak je 0, pouzije sa CHARGER_VREF_MV.
#ifndef CHARGER_USE_VREFINT
#define CHARGER_USE_VREFINT 1
#endif

#ifndef CHARGER_ADC_CHANNEL
#define CHARGER_ADC_CHANNEL ADC_CHANNEL_6
#endif

#ifndef CHARGER_VREF_MV
#define CHARGER_VREF_MV 3100u
#endif

// Delic: BAT-470k-ADC-47k-GND
#ifndef CHARGER_RTOP_OHM
#define CHARGER_RTOP_OHM 470000u
#endif

#ifndef CHARGER_RBOT_OHM
#define CHARGER_RBOT_OHM 47000u
#endif

// Zhluk informacii pre zobrazenie na displayi alebo pre aplikacnu logiku.
typedef struct
{
    int32_t battery_mv;
    float   battery_v;
    bool    battery_valid;
    bool    vin_present;
    bool    charge_allowed;
    bool    charge_enabled;
    bool    is_charging;
    bool    battery_low;
    bool    battery_empty;
    bool    battery_full;
    bool    restore_allowed;
    int     status;
} charger_info_t;

#ifdef __cplusplus
extern "C" {
#endif

// Inicializacia charger modulu.
// Driver vzdy nastavi CEN=1 a finalny stav doriesi az po uplynuti hold casu.
void charger_init(void);

// Pravidelna obsluha charger modulu.
// Zavolaj ju z main loop a posli aktualny stav VIN/VBUS.
void charger_task(bool vin_present);

// Globalne povolenie alebo zakaz nabijania z aplikacie.
// Boot/VIN hold ostava aktivny aj ked je nabijanie zakazane.
void charger_set_charge_allowed(bool allow);
bool charger_get_charge_allowed(void);

// Vrati posledny zmerany a vyhodnoteny stav.
void charger_get_info(charger_info_t *info);
float charger_get_battery_voltage(void);

bool charger_is_battery_low(void);
bool charger_is_battery_empty(void);
bool charger_should_sleep(void);
bool charger_should_restore(void);

// Po wake zo STOP2 vynuti nove meranie baterie pri dalsom charger_task().
void charger_force_measure(void);

// Nizkourovnove ovladanie CEN pinu.
void charger_set_enabled(bool enable);

// Stav CHG/CEN pre rychlu diagnostiku.
int charger_get_status_fast(bool vin_present);
int charger_get_status_blocking(bool vin_present, uint32_t observe_ms);
bool charger_is_charging_fast(void);

// Okamzite meranie baterie v mV. Pri vypnutom ADC vrati -1.
int32_t charger_read_bat(void);

#ifdef __cplusplus
}
#endif

#endif // CHARGER_H
#endif /* CHARGER_CHARGER_H_ */
