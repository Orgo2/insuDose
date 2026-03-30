/*
 * clock.c
 *
 *  Created on: Nov 8, 2025
 *      Author: orgo
 */

#include "clock.h"
#include "rtc.h"
#include "stm32wbxx_hal.h"

// Pomocná funkcia na konverziu z BIN na BCD
static uint8_t BIN2BCD(uint8_t value) {
    return ((value / 10) << 4) | (value % 10);
}

// Pomocná funkcia na konverziu z BCD na BIN
static uint8_t BCD2BIN(uint8_t value) {
    return ((value >> 4) * 10) + (value & 0x0F);
}

// Inicializácia RTC hodín s externým 32.768 kHz kryštálom
void Clock_Init(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    
    // Povolíme prístup k RTC registrom
    HAL_PWR_EnableBkUpAccess();
    
    // Konfigurácia LSE (Low Speed External oscillator - 32.768 kHz)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    // Nastavenie LSE ako zdroja hodín pre RTC
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    // Povolíme RTC hodiny
    __HAL_RCC_RTC_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
    
    // Inicializácia RTC handle
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;  // Pre LSE 32.768 kHz: (127+1)*(255+1) = 32768
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        Error_Handler();
    }
    
    // Nastavenie defaultného času (00:00:00)
    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
        Error_Handler();
    }
    
    // Nastavenie defaultného dátumu (01.01.2000)
    sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 0x1;
    sDate.Year = 0x0;
    
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
        Error_Handler();
    }
}

// Nastavenie času a dátumu (hh:mm:ss DD:MM:RR)
void SetClock(uint8_t hours, uint8_t minutes, uint8_t seconds, 
              uint8_t day, uint8_t month, uint8_t year) {
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    
    // Nastavenie času
    sTime.Hours = BIN2BCD(hours);
    sTime.Minutes = BIN2BCD(minutes);
    sTime.Seconds = BIN2BCD(seconds);
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
        Error_Handler();
    }
    
    // Nastavenie dátumu
    sDate.Date = BIN2BCD(day);
    sDate.Month = BIN2BCD(month);
    sDate.Year = BIN2BCD(year);
    
    // Výpočet dňa v týždni (Zeller's congruence pre Gregoriánsky kalendár)
    uint16_t full_year = 2000 + year;
    uint8_t m = month;
    uint16_t y = full_year;
    if (m < 3) {
        m += 12;
        y--;
    }
    uint8_t weekday = (day + (13 * (m + 1)) / 5 + y + y / 4 - y / 100 + y / 400) % 7;
    // Konverzia na RTC formát (Monday = 1, Sunday = 7)
    weekday = (weekday == 0) ? 7 : weekday;
    sDate.WeekDay = weekday;
    
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
        Error_Handler();
    }
}

// Získanie času a dátumu
DateTime_t GetClock(void) {
    DateTime_t dateTime;
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    
    // Získanie času
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
    // Získanie dátumu (musí sa volať po GetTime)
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
    
    // Konverzia z BCD na BIN
    dateTime.hours = BCD2BIN(sTime.Hours);
    dateTime.minutes = BCD2BIN(sTime.Minutes);
    dateTime.seconds = BCD2BIN(sTime.Seconds);
    dateTime.day = BCD2BIN(sDate.Date);
    dateTime.month = BCD2BIN(sDate.Month);
    dateTime.year = BCD2BIN(sDate.Year);
    
    return dateTime;
}


