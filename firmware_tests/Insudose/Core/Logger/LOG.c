/*
#include "LOG.h"
#include "main.h"
#include "stm32wbxx_hal.h"
#include <string.h>
#include "usbd_core.h"  // aby si videl hUsbDeviceFS
//#include "usbd_msc.h"   // aby si videl typ triedy MSC

extern RTC_HandleTypeDef hrtc;

#define LOG_FLASH_START_ADDRESS  0x08080000UL
#define LOG_FLASH_SIZE            (24 * 1024UL)
#define LOG_FLASH_END_ADDRESS    (LOG_FLASH_START_ADDRESS + LOG_FLASH_SIZE - 1)
//#define LOG_RECORD_SIZE          16

typedef struct {
    uint8_t day;
    uint8_t month;
    uint8_t year;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t dose;
    int8_t temperature;
    uint8_t reserved[6];
} LogRecord;

void Log_SaveRecord(uint8_t dose, int8_t temperature)
{
	/////if usb is connected do not write////////////////
	extern USBD_HandleTypeDef hUsbDeviceFS;

	if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
	    return;  // USB je pripojené, nerob flash zápis
	}
	//////////////end of USB detection///////////////
	RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    LogRecord record;
    record.day = sDate.Date;
    record.month = sDate.Month;
    record.year = sDate.Year;
    record.hour = sTime.Hours;
    record.minute = sTime.Minutes;
    record.second = sTime.Seconds;
    record.dose = dose;
    record.temperature = temperature;
    memset(record.reserved, 0xFF, sizeof(record.reserved));

    HAL_FLASH_Unlock();

    uint32_t address = LOG_FLASH_START_ADDRESS;
    uint32_t first_free_address = LOG_FLASH_START_ADDRESS;
    uint8_t found_free = 0;
    volatile uint8_t g_usb_active = 0; //is flash connected?

    while (address <= LOG_FLASH_END_ADDRESS)
    {
        uint32_t *p = (uint32_t *)address;
        if (p[0] == 0xFFFFFFFF && p[1] == 0xFFFFFFFF &&
            p[2] == 0xFFFFFFFF && p[3] == 0xFFFFFFFF)
        {
            first_free_address = address;
            found_free = 1;
            break;
        }
        address += LOG_RECORD_SIZE;
    }

    if (!found_free)
    {
        first_free_address = LOG_FLASH_START_ADDRESS;
    }

    uint32_t page_start = first_free_address & ~(FLASH_PAGE_SIZE - 1);
    uint8_t need_erase = 0;

    for (uint32_t addr = page_start; addr < (page_start + FLASH_PAGE_SIZE); addr += 4)
    {
        if (*(uint32_t *)addr != 0xFFFFFFFF)
        {
            need_erase = 1;
            break;
        }
    }

    if (need_erase)
    {
        FLASH_EraseInitTypeDef eraseInitStruct;
        uint32_t PageError;

        eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        eraseInitStruct.Page = (page_start - 0x08000000) / FLASH_PAGE_SIZE;
        eraseInitStruct.NbPages = 1;

        if (HAL_FLASHEx_Erase(&eraseInitStruct, &PageError) != HAL_OK)
        {
            HAL_FLASH_Lock();
            Error_Handler();
        }
    }

    uint64_t *pData = (uint64_t *)&record;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, first_free_address, pData[0]) != HAL_OK)
    {
        HAL_FLASH_Lock();
        Error_Handler();
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, first_free_address + 8, pData[1]) != HAL_OK)
    {
        HAL_FLASH_Lock();
        Error_Handler();
    }

    HAL_FLASH_Lock();
}

// Funkcia na nacitanie zaznamu
uint8_t Log_LoadRecord(uint32_t index, LogRecord *rec)
{
    uint32_t address = LOG_FLASH_START_ADDRESS + index * LOG_RECORD_SIZE;

    if (address > LOG_FLASH_END_ADDRESS)
        return 0; // Index mimo rozsah

    uint32_t *p = (uint32_t *)address;
    if (p[0] == 0xFFFFFFFF && p[1] == 0xFFFFFFFF &&
        p[2] == 0xFFFFFFFF && p[3] == 0xFFFFFFFF)
    {
        return 0; // prazdne miesto
    }

    memcpy(rec, (void *)address, sizeof(LogRecord));
    return 1; // uspesne nacitane
}

// Funkcia na vymazanie celeho logu
void Log_ClearAll(void)
{
	/////if usb is connected do not write////////////////
	extern USBD_HandleTypeDef hUsbDeviceFS;

	if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
	    return;  // USB je pripojené, nerob flash zápis
	}
	//////////////end of USB detection///////////////
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t PageError;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.Page = (LOG_FLASH_START_ADDRESS - 0x08000000) / FLASH_PAGE_SIZE;
    eraseInitStruct.NbPages = LOG_FLASH_SIZE / FLASH_PAGE_SIZE;

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        Error_Handler();
    }

    HAL_FLASH_Lock();
}
*/
