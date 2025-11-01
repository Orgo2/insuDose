#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "app_fatfs.h"
#include <stdio.h>
#include <string.h>
#include "ff.h"
#include "main.h"
#include "stm32wbxx_hal.h"

/////definujeme velkost RAM disku////
#define SECTOR_SIZE    512
#define NUM_SECTORS    128
////definujeme subor////
#define LOG_FILE_NAME    "log.txt"
#define LOG_FILE_SIZE    (SECTOR_SIZE * NUM_SECTORS)  // 32 KB
#define MAX_LOG_LINE_LEN 64	// max. dĺžka jedného riadku
// Sentinel hodnoty pre "dáta nie sú k dispozícii"
#define LOG_NO_YEAR  0xFFFF      // year chýba → čas sa zapíše ako 12x '/'
#define LOG_NO_U8    0xFF        // chýbajúci mesiac/deň/hodina/minúta/dávka
#define LOG_NO_TEMP  ((int8_t)0x80) // -128 označí chýbajúcu teplotu
// --- RAMDISK → FLASH snapshot / restore -------------------------------
#ifndef LOGGER_FLASH_PAGE_SIZE
#define LOGGER_FLASH_PAGE_SIZE   4096u       // WB55: 4 kB page
#endif

#ifndef LOGGER_FLASH_END
#define LOGGER_FLASH_END         0x08080000u // WB55CG 512 kB: koniec FLASH
#endif

// koľko bajtov potrebujeme uložiť (celý RAM disk)
#define LOGGER_BACKUP_BYTES      (SECTOR_SIZE * NUM_SECTORS)

// Počet strán potrebných na header + dáta
#define LOGGER_BACKUP_PAGES      (((LOGGER_BACKUP_BYTES + 32u + LOGGER_FLASH_PAGE_SIZE - 1u) / LOGGER_FLASH_PAGE_SIZE))

// Prototypy
bool logger_persist_ramdisk_to_flash(void);
bool logger_restore_ramdisk_from_flash(void);
extern uint8_t ram_disk[SECTOR_SIZE * NUM_SECTORS];



// Inicializácia RAMDISKu a FATFS
extern uint32_t log_offset;
bool init_ramdisk(void);

// Pridanie jedného záznamu do rotujúceho logu

void append_log_rotating(uint16_t year, uint8_t mon, uint8_t day,
                         uint8_t hour, uint8_t min,
                         uint8_t davka, int8_t teplota_c);
////pridaana detekcia ramdisku////
bool logger_ramdisk_is_empty(void);
void logger_invalidate_ramdisk(void);

#endif // LOGGER_H
