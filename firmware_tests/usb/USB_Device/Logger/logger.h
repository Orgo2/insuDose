#ifndef LOGGER_H
#define LOGGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_fatfs.h"
#include "ff.h"
#include "main.h"
#include "stm32wbxx_hal.h"
#include "../../Drivers/RTC/rtc_driver.h"

// Velkost RAM disku sa nemení, FATFS na nej uz zavisi.
#define SECTOR_SIZE       512u
#define NUM_SECTORS       128u
#define LOG_FILE_NAME     "0:/log.txt"
#define LOG_FILE_SIZE     (SECTOR_SIZE * NUM_SECTORS)  // 64 kB
#define MAX_LOG_LINE_LEN  256u

// Sentinel hodnoty pre polia, ktore v danom zazname nemusia byt k dispozicii.
#define LOG_NO_U8    0xFFu
#define LOG_NO_TEMP  ((int8_t)0x80)

// Snapshot RAM disku do internej FLASH.
#ifndef LOGGER_FLASH_PAGE_SIZE
#define LOGGER_FLASH_PAGE_SIZE   FLASH_PAGE_SIZE
#endif

#ifndef LOGGER_FLASH_END
#define LOGGER_FLASH_END         0x08080000u
#endif

#ifndef LOGGER_SNAPSHOT_SLOT_COUNT
#define LOGGER_SNAPSHOT_SLOT_COUNT 2u
#endif

#ifndef LOGGER_SNAPSHOT_METADATA_MAX_BYTES
#define LOGGER_SNAPSHOT_METADATA_MAX_BYTES 64u
#endif

#define LOGGER_BACKUP_BYTES      (SECTOR_SIZE * NUM_SECTORS)

extern uint8_t ram_disk[SECTOR_SIZE * NUM_SECTORS];
extern uint32_t log_offset;

bool init_ramdisk(void);
bool logger_mount_ramdisk(void);

// Logger berie cas a datum priamo z RTC. Ak RTC este nema validny datum/cas,
// zapise namiesto timestampu 12 lomitok. Prve pole je poradove cislo zaznamu.
bool append_log_rotating(uint8_t davka, int8_t teplota_c);
bool append_log_scan(const uint16_t *avg, const uint16_t *bg, uint32_t nbins,
                     uint32_t total_bins, const char *label);

bool logger_persist_ramdisk_to_flash(void);
bool logger_restore_ramdisk_from_flash(void);
bool logger_flash_snapshot_is_available(void);
void logger_set_snapshot_metadata(const void *data, uint16_t len);
bool logger_get_snapshot_metadata(void *data, uint16_t max_len, uint16_t *out_len);

bool logger_ramdisk_is_empty(void);
void logger_invalidate_ramdisk(void);
uint32_t logger_get_last_flash_error(void);

void logger_set_usb_session_active(bool active);

#endif /* LOGGER_H */
