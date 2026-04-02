#ifndef DRIVERS_TMP102_TMP102_H_
#define DRIVERS_TMP102_TMP102_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "../RTC/rtc_driver.h"

// TMP102 7-bit I2C adresa pri ADD0 = V+.
#ifndef TMP102_I2C_ADDR
#define TMP102_I2C_ADDR                        (0x49u)
#endif

// Po zapnuti napajania a konfiguracie pockaj na prvu istu konverziu.
#ifndef TMP102_POWER_SETTLE_MS
#define TMP102_POWER_SETTLE_MS                 (25u)
#endif

// Najnizsia uz nebezpecna teplota pre Apidra/Toujeo pero.
// Hodnota je v desatinach stupna C, 250 = 25.0 C.
#ifndef TMP102_OVERTEMP_ON_dC
#define TMP102_OVERTEMP_ON_dC                  (250)
#endif

// Hystereza pre ukoncenie alarmu.
#ifndef TMP102_OVERTEMP_OFF_dC
#define TMP102_OVERTEMP_OFF_dC                 (240)
#endif

// Najnizsia conversion rate, pri ktorej zostava ALERT aktivny autonomne.
#ifndef TMP102_CONVERSION_PERIOD_MS
#define TMP102_CONVERSION_PERIOD_MS            (4000u)
#endif

// Ako casto pocas aktivneho prehrievania precitat teplotu a aktualizovat maximum.
#ifndef TMP102_OVERTEMP_SAMPLE_PERIOD_S
#define TMP102_OVERTEMP_SAMPLE_PERIOD_S        (60u)
#endif

// Ako casto mimo alarmu obnovit poslednu teplotu pre log davky.
#ifndef TMP102_IDLE_SAMPLE_PERIOD_MS
#define TMP102_IDLE_SAMPLE_PERIOD_MS           (60000u)
#endif

#ifndef TMP102_RETRY_PERIOD_MS
#define TMP102_RETRY_PERIOD_MS                 (1000u)
#endif

#ifndef TMP102_I2C_TIMEOUT_MS
#define TMP102_I2C_TIMEOUT_MS                  (5u)
#endif

#define TMP102_BACKUP_VERSION                  (1u)
#define TMP102_BACKUP_FLAG_MAX_VALID           (1u << 0)
#define TMP102_BACKUP_FLAG_OVERTEMP_ACTIVE     (1u << 1)
#define TMP102_BACKUP_FLAG_OVERTEMP_START_VALID (1u << 2)
#define TMP102_BACKUP_FLAG_LAST_END_VALID      (1u << 3)

typedef struct
{
    uint32_t version;
    uint32_t flags;
    uint32_t overtemp_time_s;
    int16_t max_temperature_dC;
    int16_t last_temperature_dC;
    rtc_datetime_t max_temperature_datetime;
    rtc_datetime_t overtemp_start_datetime;
    rtc_datetime_t last_overtemp_end_datetime;
} tmp102_backup_t;

typedef struct
{
    bool powered;
    bool configured;
    bool enabled;
    bool alarm_active;
    bool temperature_valid;
    bool max_temperature_valid;
    bool overtemp_start_valid;
    bool last_overtemp_end_valid;
    int16_t temperature_dC;
    int16_t max_temperature_dC;
    uint32_t overtemp_time_s;
    rtc_datetime_t max_temperature_datetime;
    rtc_datetime_t overtemp_start_datetime;
    rtc_datetime_t last_overtemp_end_datetime;
} tmp102_status_t;

// Inicializuje driver nad uz vytvorenym HAL I2C handle.
bool tmp102_init(I2C_HandleTypeDef *i2c);

// Povoli alebo zakaze monitoring. Pri false sa odpoji napajanie senzora.
void tmp102_set_enabled(bool enabled);

// Periodicke spracovanie stavu senzora.
void tmp102_task(void);

// Zavola sa z EXTI callbacku pri zmene stavu ALERT pinu.
void tmp102_notify_alert_irq(void);

// Urobi priame citanie teplotneho registra cez I2C s timeoutom a pri uspechu
// aktualizuje internu cache. Ak meranie zlyha, caller ma pouzit "/" v logu/UI.
bool tmp102_read_temperature_now(int16_t *temperature_dC);
bool tmp102_read_temperature_now_rounded(int8_t *temperature_c);

// Vrati poslednu uspesne nacitanu teplotu.
bool tmp102_get_last_temperature(int16_t *temperature_dC);
bool tmp102_get_last_temperature_rounded(int8_t *temperature_c);

// Celkovy cas prehrievania od posledneho resetu, v sekundach.
uint32_t tmp102_get_overtemp_time_s(void);

// Vynuluje alarm metadata pri vymene inzulinu.
void tmp102_alarm_reset(void);

// Zaloha a obnova alarm metadata do recovery snapshotu.
void tmp102_export_backup(tmp102_backup_t *backup);
bool tmp102_import_backup(const tmp102_backup_t *backup);

// Stav pre UI alebo diagnostiku.
bool tmp102_get_status(tmp102_status_t *status);

#endif /* DRIVERS_TMP102_TMP102_H_ */
