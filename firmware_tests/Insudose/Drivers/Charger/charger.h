/*
 * charger.h
 *
 *  Created on: Aug 14, 2025
 *      Author: Orgo and chatgpt
 */


/*
 * charger.h
 * Minimalne HAL-only API pre STNS01 nabíjač + dohľad batérie cez ADC watchdog.
 *
 * Dôležité elektrické poznámky:
 *  - CEN (Chip ENable) pín je na STNS01 typicky **active-HIGH**:
 *      CEN=1 -> povolený nabíjač, CEN=0 -> zakázaný nabíjač.
 *
 *
 *  - CHG (Charge status) je zvyčajne **open-drain výstup** z čipu:
 *      CHG = LOW  -> práve nabíja
 *      CHG = HIGH -> nenabíja (plné / standby / VIN nie je / CEN=0)
 *
 *
 *  - VBAT meranie:
 *      Delič: BAT — 470k — ADC — 47k — GND
 *      ADC preto vidí len zlomok napätia (Rbot/(Rtop+Rbot)), treba matematicky prepočítať.
 *      Delič je “veľmi mäkký” (veľká impedancia), preto používame dlhší sampling čas ADC,
 *      napr. 640.5 cyklov.
 *
 *  - Spotreba:
 *      ADC beží len pri jednorazovom meraní (fast) a keď je “ozbrojený” watchdog blízko prahu.
 *      Inak je ADC zastavený, aby sa šetrila energia.
 */

#ifndef CHARGER_H
#define CHARGER_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#pragma once

/* -------------------- Piny (uprav podľa dosky) -------------------- */
#define CHARGER_CEN_GPIO_Port   GPIOB
#define CHARGER_CEN_Pin         GPIO_PIN_0    // PB0 -> CEN (active-HIGH)

#define CHARGER_CHG_GPIO_Port   GPIOA
#define CHARGER_CHG_Pin         GPIO_PIN_10   // PA10 -> CHG (active-LOW, open-drain, vyžaduje pull-up)

#define CHARGER_BAT_ADC_Port    GPIOA
#define CHARGER_BAT_ADC_Pin     GPIO_PIN_1    // PA1  -> VBAT delič (470k/47k)

/* -------------------- Logika úrovní -------------------- */
#define CHARGER_CEN_ACTIVE_HIGH 1
#define CHARGER_CHG_ACTIVE_LOW  1

/* -------------------- Stavy -------------------- */
typedef enum {
    CHARGER_STATUS_IDLE     = 0,  // CHG=HIGH  (nenabíja)
    CHARGER_STATUS_CHARGING = 1,  // CHG=LOW   (nabíja)
    CHARGER_STATUS_FAULT    = 3   // toggling CHG ~1 Hz (zistené len v blocking volaní)
} CHARGER_Status;

/* -------------------- ADC zap/vyp na projekte -------------------- */
#ifndef CHARGER_USE_ADC
#define CHARGER_USE_ADC 1
#endif

/* Delič: BAT—470k—ADC—47k—GND  */
#ifndef CHARGER_RTOP_OHM
#define CHARGER_RTOP_OHM 470000
#endif
#ifndef CHARGER_RBOT_OHM
#define CHARGER_RBOT_OHM 47000
#endif

/* ADC kanál pre PA1 – uprav podľa .ioc (napr. ADC_CHANNEL_6) */
#define CHARGER_ADC_CHANNEL ADC_CHANNEL_6

/* Výstup „fast“ volania: okamžitý status + VBAT v mV (alebo -1, ak ADC nepoužívaš) */
typedef struct {
    int32_t        batt_mv;   // mV, alebo -1 ak ADC nepoužívaš
    CHARGER_Status status;    // CHARGING / IDLE (bez čakania)
} CHARGER_Fast;

/* -------------------- API: charger GPIO -------------------- */

/* Nastaví CEN pin:
 *  enable=true  -> povolí nabíjač
 *  enable=false -> zakáže nabíjač
 * Pozn.: len prepis pinu, žiadna “magická” inicializácia. GPIO si nastav v CubeMX. */
void          charger_set_enabled(bool enable);

/* FAST (rýchle) čítanie bez čakania:
 *  - prečíta CHG (status nabíjania)
 *  - jednorazovo zmeria VBAT (ak je povolený ADC) a vráti v mV
 *  - vyhodnotí politiku nízkej batérie a prípadne armuje/odarmuje ADC watchdog
 *    (pozri charger_batt_watchdog_config() nižšie)
 */
CHARGER_Fast  charger_get_fast(void);

/* BLOCKING (pomalé) čítanie s detekciou FAULT:
 *  - fixne čaká ~1200 ms na toggling CHG (fault ~1 Hz z datasheetu)
 *  - vráti CHARGER_STATUS_FAULT pri detekcii; inak CHARGING/IDLE
 */
CHARGER_Status charger_get_status_blocking(void);

/* -------------------- Low-battery politika + watchdog -------------------- */

/* Nastavenie politík (v mV):
 *  sleep_mv ... úroveň batérie pre “ísť spať”
 *  hyst_mv  ... hysterézia pre “prebudenie” (prebudí na sleep_mv + hyst_mv)
 *  bwdg_mv  ... “okolie” okolo sleep_mv, v ktorom SA ARMUJE ADC watchdog
 *               (ak VBAT <= sleep_mv + bwdg_mv), aby sa neprepásol prudký pád
 */
void charger_batt_watchdog_config(uint32_t sleep_mv, uint32_t hyst_mv, uint32_t bwdg_mv);

/* Číta flag, ktorý nastavuje IRQ watchdogu:
 *  true  -> batéria nízka, systém by sa mal uložiť (logger flush, “discharged” na displej, sleep)
 *  false -> batéria OK / alebo sme už nad hysteréziou (prebudené)
 */
bool charger_batt_low_flag(void);

/* Ručne odarmuje watchdog a zastaví ADC (šetrenie spotreby).
 * Použi napr. po úspešnom prechode do SLEEP, ak nechceš, aby ADC bežal. */
void charger_batt_watchdog_disarm(void);

#endif // CHARGER_H
