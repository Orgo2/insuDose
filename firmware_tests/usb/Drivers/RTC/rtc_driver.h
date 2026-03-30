#ifndef DRIVERS_RTC_RTC_DRIVER_H_
#define DRIVERS_RTC_RTC_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

typedef struct
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} rtc_time_t;

typedef struct
{
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  weekday;
} rtc_date_t;

typedef struct
{
    rtc_date_t date;
    rtc_time_t time;
} rtc_datetime_t;

// Inicializuje pristup k Cube/HAL RTC handle-u.
bool rtc_driver_init(void);

// Samostatne nastavovanie casu a datumu.
bool rtc_driver_set_time(const rtc_time_t *time);
bool rtc_driver_set_date(const rtc_date_t *date);
bool rtc_driver_set_datetime(const rtc_datetime_t *datetime);

// Samostatne citanie casu a datumu.
bool rtc_driver_get_time(rtc_time_t *time);
bool rtc_driver_get_date(rtc_date_t *date);
bool rtc_driver_get_datetime(rtc_datetime_t *datetime);

// Vrati true len ked bol datum/cas explicitne nastaveny.
bool rtc_driver_has_valid_datetime(void);

// Pri novom builde alebo po strate backup domeny nastavi RTC na cas kompilacie
// firmware. Pri dalsich resetoch uz cas neprepisuje a RTC bezi dalej.
bool rtc_driver_sync_build_datetime(void);

// Prevody pre FatFs timestamp.
uint32_t rtc_driver_get_fattime(void);

// Wakeup timer pre STOP2 polling.
// period_seconds je 1..65536 sekund.
bool rtc_driver_start_wakeup(uint32_t period_seconds);
void rtc_driver_stop_wakeup(void);
void rtc_driver_wakeup_irq_handler(void);

#endif /* DRIVERS_RTC_RTC_DRIVER_H_ */
