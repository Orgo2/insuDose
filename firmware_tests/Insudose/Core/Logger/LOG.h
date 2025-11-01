/* log.h */
/*
#ifndef __LOG_H
#define __LOG_H

#include "stm32wbxx_hal.h"

// Konfigurácia FLASH pre logovanie
#define LOG_FLASH_START_ADDRESS   0x080F8000UL    // Začiatok oblasti v pamäti
#define LOG_FLASH_SIZE_BYTES   (24 * 1024)     // 24 kB flash
#define LOG_RECORD_SIZE        16               // Veľkosť 1 záznamu v bajtoch
#define LOG_MAX_RECORDS        (LOG_FLASH_SIZE_BYTES / LOG_RECORD_SIZE)

// Štruktúra jedného záznamu
typedef struct {
    uint8_t day;
    uint8_t month;
    uint8_t year;         // Rok (posledné dve číslice)
    uint8_t hour;
    uint8_t minute;
    uint8_t dose;         // Dávka inzulínu
    int8_t temperature;   // Teplota v °C (signed 8-bit)
} __attribute__((packed)) LogRecord_t;

// API funkcie
void Log_Init(void);
void Log_SaveRecord(uint8_t dose, int8_t temperature);
HAL_StatusTypeDef Log_ReadRecord(uint32_t index, LogRecord_t *record);
void Log_EraseArea(void);

#endif /* __LOG_H */
*/
