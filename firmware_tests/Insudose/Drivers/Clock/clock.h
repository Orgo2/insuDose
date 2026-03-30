/*
 * clock.h
 *
 *  Created on: Nov 8, 2025
 *      Author: orgo
 */

#ifndef CLOCK_CLOCK_H_
#define CLOCK_CLOCK_H_

#include <stdint.h>

// Štruktúra pre dátum a čas
typedef struct {
    uint8_t hours;      // 0-23
    uint8_t minutes;    // 0-59
    uint8_t seconds;    // 0-59
    uint8_t day;        // 1-31
    uint8_t month;      // 1-12
    uint8_t year;       // 0-99 (20xx)
} DateTime_t;

// Inicializácia RTC hodín s externým 32.768 kHz kryštálom
void Clock_Init(void);

// Nastavenie času a dátumu (hh:mm:ss DD:MM:RR)
void SetClock(uint8_t hours, uint8_t minutes, uint8_t seconds, 
              uint8_t day, uint8_t month, uint8_t year);

// Získanie času a dátumu
DateTime_t GetClock(void);

#endif /* CLOCK_CLOCK_H_ */
