#include "logger.h"

#include "stm32wbxx_hal_flash.h"
#include "stm32wbxx_hal_flash_ex.h"

/*
 * Log storage module.
 * The live log lives on a RAM disk backed by FatFS. When the device is about
 * to sleep on an empty battery, the whole RAM disk plus selected metadata are
 * snapshotted into internal flash so the next boot can restore the previous
 * state. During an active USB MSC session, local writes are blocked because
 * the host owns the medium.
 */

/* Magic word that marks one valid flash snapshot slot header. */
#define LOGGER_SNAPSHOT_MAGIC         0x32534452u  // 'RDS2'
/* Snapshot layout version; bump when on-flash format changes. */
#define LOGGER_SNAPSHOT_VERSION       4u
/* Header flag meaning the log file already wrapped around its fixed-size area. */
#define LOGGER_SNAPSHOT_FLAG_WRAPPED  0x00000001u
/* Synthetic error code used when written flash data cannot be verified back. */
#define LOGGER_FLASH_ERROR_VERIFY     0x80000000u

typedef struct
{
    /* Signature and format checks for one snapshot slot. */
    uint32_t magic;
    uint32_t version;
    /* Monotonic sequence number used to choose the newest slot. */
    uint32_t sequence;
    /* Length of the RAM-disk payload stored after the metadata area. */
    uint32_t data_len;
    /* Number of valid metadata bytes stored in s_snapshot_metadata. */
    uint32_t metadata_len;
    /* Current write cursor inside the circular log file. */
    uint32_t log_offset;
    /* Last issued log record number. */
    uint32_t record_counter;
    /* Bit flags such as LOGGER_SNAPSHOT_FLAG_WRAPPED. */
    uint32_t flags;
    /* CRC over header payload, metadata and RAM-disk image. */
    uint32_t crc32;
} logger_snapshot_header_t;

/* Aligned header size written to each flash snapshot slot. */
#define LOGGER_SNAPSHOT_HEADER_SIZE   (((uint32_t)sizeof(logger_snapshot_header_t) + 7u) & ~7u)
/* Reserved aligned area for arbitrary metadata stored next to the RAM disk. */
#define LOGGER_SNAPSHOT_METADATA_SIZE (((uint32_t)LOGGER_SNAPSHOT_METADATA_MAX_BYTES + 7u) & ~7u)
/* Byte offset from slot start to the RAM-disk payload. */
#define LOGGER_SNAPSHOT_DATA_OFFSET   (LOGGER_SNAPSHOT_HEADER_SIZE + LOGGER_SNAPSHOT_METADATA_SIZE)
/* Number of flash pages needed for one snapshot slot. */
#define LOGGER_SNAPSHOT_SLOT_PAGES    (((LOGGER_BACKUP_BYTES + LOGGER_SNAPSHOT_DATA_OFFSET) + LOGGER_FLASH_PAGE_SIZE - 1u) / LOGGER_FLASH_PAGE_SIZE)
/* Full byte size consumed by one snapshot slot in flash. */
#define LOGGER_SNAPSHOT_SLOT_BYTES    (LOGGER_SNAPSHOT_SLOT_PAGES * LOGGER_FLASH_PAGE_SIZE)

/* True while USB MSC owns the RAM disk and firmware-local file writes must stop. */
static volatile bool s_usb_session_active = false;
/* True after the fixed-size log file started overwriting from the beginning. */
static bool s_wrapped = false;
/* Opaque metadata blob saved next to the RAM-disk snapshot (for TMP102 backup etc.). */
static uint8_t s_snapshot_metadata[LOGGER_SNAPSHOT_METADATA_MAX_BYTES] = {0};
/* Number of valid bytes currently stored in s_snapshot_metadata. */
static uint16_t s_snapshot_metadata_len = 0u;
/* Last used monotonic record number written to the CSV log. */
static uint32_t s_record_counter = 0u;
/* Last flash-programming or verification error observed by snapshot code. */
static uint32_t s_last_flash_error = HAL_FLASH_ERROR_NONE;

/* Current circular write cursor inside the on-RAM CSV log file. */
uint32_t log_offset = 0;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len);
static uint32_t logger_snapshot_slot_address(uint32_t slot_index);
static uint32_t logger_snapshot_crc(const logger_snapshot_header_t *header,
                                    const uint8_t *metadata,
                                    const uint8_t *data);
static bool logger_read_snapshot_slot(uint32_t slot_index, bool load_metadata, logger_snapshot_header_t *header, const uint8_t **data);
static bool logger_find_latest_snapshot(logger_snapshot_header_t *header, const uint8_t **data, uint32_t *slot_index);
static bool flash_erase_pages(uint32_t addr_start, uint32_t pages);
static bool flash_program_bytes(uint32_t dst, const uint8_t *src, uint32_t len);
static bool logger_prepare_for_write(FIL *file);
static bool logger_append_line(const char *line);
static void logger_write_2digits(char *dst, unsigned value);
static void logger_write_4digits(char *dst, unsigned value);
static void logger_format_timestamp_12(char *dst, const rtc_datetime_t *datetime);

/* Called by app_runtime when the host starts/stops owning the RAM disk over USB MSC. */
void logger_set_usb_session_active(bool active)
{
    s_usb_session_active = active;
}

uint32_t logger_get_last_flash_error(void)
{
    return s_last_flash_error;
}

void logger_set_snapshot_metadata(const void *data, uint16_t len)
{
    if ((data == NULL) || (len == 0u)) {
        memset(s_snapshot_metadata, 0, sizeof(s_snapshot_metadata));
        s_snapshot_metadata_len = 0u;
        return;
    }

    if (len > LOGGER_SNAPSHOT_METADATA_MAX_BYTES) {
        len = LOGGER_SNAPSHOT_METADATA_MAX_BYTES;
    }

    memcpy(s_snapshot_metadata, data, len);
    if (len < LOGGER_SNAPSHOT_METADATA_MAX_BYTES) {
        memset(&s_snapshot_metadata[len], 0, LOGGER_SNAPSHOT_METADATA_MAX_BYTES - len);
    }
    s_snapshot_metadata_len = len;
}

bool logger_get_snapshot_metadata(void *data, uint16_t max_len, uint16_t *out_len)
{
    if ((data == NULL) || (s_snapshot_metadata_len == 0u) || (max_len < s_snapshot_metadata_len)) {
        if (out_len != NULL) {
            *out_len = s_snapshot_metadata_len;
        }
        return false;
    }

    memcpy(data, s_snapshot_metadata, s_snapshot_metadata_len);
    if (out_len != NULL) {
        *out_len = s_snapshot_metadata_len;
    }
    return true;
}

/* Create a fresh FAT volume and empty log file directly on the RAM disk buffer. */
bool init_ramdisk(void)
{
    FRESULT res;
    static BYTE work[SECTOR_SIZE];
    FIL file;

    log_offset = 0;
    s_wrapped = false;
    s_record_counter = 0u;
    memset(s_snapshot_metadata, 0, sizeof(s_snapshot_metadata));
    s_snapshot_metadata_len = 0u;
    memset(ram_disk, 0, sizeof(ram_disk));

    (void)f_mount(NULL, "0:", 0);

    res = f_mkfs("0:", FM_FAT | FM_SFD, 0, work, sizeof(work));
    if (res != FR_OK) {
        return false;
    }

    if (f_mount(&USERFatFs, "0:", 1) != FR_OK) {
        return false;
    }

    res = f_open(&file, LOG_FILE_NAME, FA_OPEN_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        return false;
    }

    (void)f_close(&file);
    return true;
}

/* Mount and validate the existing FAT volume already present in RAM. */
bool logger_mount_ramdisk(void)
{
    FIL file;

    if (f_mount(&USERFatFs, "0:", 1) != FR_OK) {
        return false;
    }

    if (f_open(&file, LOG_FILE_NAME, FA_OPEN_ALWAYS | FA_WRITE) != FR_OK) {
        return false;
    }

    (void)f_close(&file);
    return true;
}

/* Format one CSV line and append it into the fixed-size circular log file. */
bool append_log_rotating(uint8_t davka, int8_t teplota_c)
{
    char line[MAX_LOG_LINE_LEN];
    char timestamp[13];
    char dosebuf[5];
    char tempbuf[6];
    uint32_t record_number;
    rtc_datetime_t datetime;
    bool timestamp_valid;

    record_number = (s_record_counter == UINT32_MAX) ? UINT32_MAX : (s_record_counter + 1u);
    timestamp_valid = rtc_driver_has_valid_datetime() && rtc_driver_get_datetime(&datetime);
    if (timestamp_valid) {
        logger_format_timestamp_12(timestamp, &datetime);
    } else {
        memset(timestamp, '/', 12);
        timestamp[12] = '\0';
    }

    if (davka != LOG_NO_U8) {
        (void)snprintf(dosebuf, sizeof(dosebuf), "%u", davka);
    } else {
        strcpy(dosebuf, "/");
    }

    if (teplota_c != LOG_NO_TEMP) {
        (void)snprintf(tempbuf, sizeof(tempbuf), "%d", teplota_c);
    } else {
        strcpy(tempbuf, "/");
    }

    (void)snprintf(line, sizeof(line), "%lu,%s,%s,%s\r\n",
                   (unsigned long)record_number,
                   timestamp,
                   dosebuf,
                   tempbuf);
    if (!logger_append_line(line)) {
        return false;
    }

    s_record_counter = record_number;
    return true;
}

/* Open the log file only when firmware, not the USB host, owns the RAM disk. */
static bool logger_prepare_for_write(FIL *file)
{
    if ((file == NULL) || s_usb_session_active || logger_ramdisk_is_empty()) {
        return false;
    }

    if (f_mount(&USERFatFs, "0:", 1) != FR_OK) {
        return false;
    }

    return (f_open(file, LOG_FILE_NAME, FA_WRITE | FA_OPEN_ALWAYS) == FR_OK);
}

/* Append one already-formatted line into the fixed-size circular CSV file. */
static bool logger_append_line(const char *line)
{
    FIL file;
    FRESULT res;
    UINT bw;
    UINT line_len;
    FSIZE_t size;
    FSIZE_t target_offset;

    if ((line == NULL) || !logger_prepare_for_write(&file)) {
        return false;
    }

    line_len = (UINT)strlen(line);
    size = f_size(&file);

    if (!s_wrapped && size < LOG_FILE_SIZE) {
        log_offset = (uint32_t)size;

        if ((size + line_len) <= LOG_FILE_SIZE) {
            target_offset = size;
        } else {
            s_wrapped = true;
            log_offset = 0;
            target_offset = 0;
        }
    } else {
        if (log_offset > (LOG_FILE_SIZE - MAX_LOG_LINE_LEN)) {
            log_offset = 0;
        }

        target_offset = log_offset;
    }

    res = f_lseek(&file, target_offset);
    if (res != FR_OK) {
        (void)f_close(&file);
        return false;
    }

    bw = 0u;
    res = f_write(&file, line, line_len, &bw);
    if ((res != FR_OK) || (bw != line_len)) {
        (void)f_close(&file);
        return false;
    }

    res = f_sync(&file);
    if (res != FR_OK) {
        (void)f_close(&file);
        return false;
    }

    log_offset = (uint32_t)(target_offset + line_len);
    if (log_offset > (LOG_FILE_SIZE - MAX_LOG_LINE_LEN)) {
        log_offset = 0;
    }

    res = f_close(&file);
    if (res != FR_OK) {
        return false;
    }

    return true;
}

static void logger_write_2digits(char *dst, unsigned value)
{
    dst[0] = (char)('0' + ((value / 10u) % 10u));
    dst[1] = (char)('0' + (value % 10u));
}

static void logger_write_4digits(char *dst, unsigned value)
{
    dst[0] = (char)('0' + ((value / 1000u) % 10u));
    dst[1] = (char)('0' + ((value / 100u) % 10u));
    dst[2] = (char)('0' + ((value / 10u) % 10u));
    dst[3] = (char)('0' + (value % 10u));
}

static void logger_format_timestamp_12(char *dst, const rtc_datetime_t *datetime)
{
    unsigned year = (unsigned)(datetime->date.year % 10000u);
    unsigned month = (unsigned)((datetime->date.month >= 1u && datetime->date.month <= 12u) ? datetime->date.month : 1u);
    unsigned day = (unsigned)((datetime->date.day >= 1u && datetime->date.day <= 31u) ? datetime->date.day : 1u);
    unsigned hours = (unsigned)((datetime->time.hours <= 23u) ? datetime->time.hours : 0u);
    unsigned minutes = (unsigned)((datetime->time.minutes <= 59u) ? datetime->time.minutes : 0u);

    logger_write_4digits(&dst[0], year);
    logger_write_2digits(&dst[4], month);
    logger_write_2digits(&dst[6], day);
    logger_write_2digits(&dst[8], hours);
    logger_write_2digits(&dst[10], minutes);
    dst[12] = '\0';
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    uint32_t i;

    crc = ~crc;

    for (i = 0; i < len; i++) {
        uint32_t value = data[i];
        uint32_t bit;

        crc ^= value;
        for (bit = 0; bit < 8u; bit++) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }

    return ~crc;
}

static uint32_t logger_snapshot_slot_address(uint32_t slot_index)
{
    return LOGGER_FLASH_END - ((LOGGER_SNAPSHOT_SLOT_COUNT - slot_index) * LOGGER_SNAPSHOT_SLOT_BYTES);
}

/* CRC covers runtime-relevant header fields, metadata and the raw RAM-disk image. */
static uint32_t logger_snapshot_crc(const logger_snapshot_header_t *header,
                                    const uint8_t *metadata,
                                    const uint8_t *data)
{
    uint32_t crc = 0u;

    crc = crc32_update(crc, (const uint8_t *)&header->sequence, sizeof(header->sequence));
    crc = crc32_update(crc, (const uint8_t *)&header->data_len, sizeof(header->data_len));
    crc = crc32_update(crc, (const uint8_t *)&header->metadata_len, sizeof(header->metadata_len));
    crc = crc32_update(crc, (const uint8_t *)&header->log_offset, sizeof(header->log_offset));
    crc = crc32_update(crc, (const uint8_t *)&header->record_counter, sizeof(header->record_counter));
    crc = crc32_update(crc, (const uint8_t *)&header->flags, sizeof(header->flags));
    if ((header->metadata_len != 0u) && (metadata != NULL)) {
        crc = crc32_update(crc, metadata, header->metadata_len);
    }
    crc = crc32_update(crc, data, header->data_len);
    return crc;
}

static bool logger_read_snapshot_slot(uint32_t slot_index, bool load_metadata, logger_snapshot_header_t *header, const uint8_t **data)
{
    uint32_t address;
    const logger_snapshot_header_t *stored_header;
    const uint8_t *stored_metadata;
    const uint8_t *stored_data;

    if (slot_index >= LOGGER_SNAPSHOT_SLOT_COUNT) {
        return false;
    }

    address = logger_snapshot_slot_address(slot_index);
    stored_header = (const logger_snapshot_header_t *)address;

    if ((stored_header->magic != LOGGER_SNAPSHOT_MAGIC) ||
        (stored_header->version != LOGGER_SNAPSHOT_VERSION) ||
        (stored_header->data_len != LOGGER_BACKUP_BYTES) ||
        (stored_header->metadata_len > LOGGER_SNAPSHOT_METADATA_MAX_BYTES) ||
        (stored_header->log_offset > LOG_FILE_SIZE) ||
        ((stored_header->flags & ~LOGGER_SNAPSHOT_FLAG_WRAPPED) != 0u)) {
        return false;
    }

    stored_metadata = (const uint8_t *)(address + LOGGER_SNAPSHOT_HEADER_SIZE);
    stored_data = (const uint8_t *)(address + LOGGER_SNAPSHOT_DATA_OFFSET);
    if (logger_snapshot_crc(stored_header, stored_metadata, stored_data) != stored_header->crc32) {
        return false;
    }

    if (load_metadata) {
        if (stored_header->metadata_len != 0u) {
            memcpy(s_snapshot_metadata, stored_metadata, stored_header->metadata_len);
        }
        if (stored_header->metadata_len < LOGGER_SNAPSHOT_METADATA_MAX_BYTES) {
            memset(&s_snapshot_metadata[stored_header->metadata_len],
                   0,
                   LOGGER_SNAPSHOT_METADATA_MAX_BYTES - stored_header->metadata_len);
        }
        s_snapshot_metadata_len = (uint16_t)stored_header->metadata_len;
    }

    if (header != NULL) {
        *header = *stored_header;
    }
    if (data != NULL) {
        *data = stored_data;
    }

    return true;
}

/* Scan all slots and choose the newest valid flash snapshot by sequence number. */
static bool logger_find_latest_snapshot(logger_snapshot_header_t *header, const uint8_t **data, uint32_t *slot_index)
{
    bool found = false;
    logger_snapshot_header_t best_header = {0};
    const uint8_t *best_data = NULL;
    uint32_t best_slot = 0u;
    uint32_t slot;

    for (slot = 0; slot < LOGGER_SNAPSHOT_SLOT_COUNT; slot++) {
        logger_snapshot_header_t candidate_header;
        const uint8_t *candidate_data;

        if (!logger_read_snapshot_slot(slot, false, &candidate_header, &candidate_data)) {
            continue;
        }

        if (!found || (candidate_header.sequence > best_header.sequence)) {
            found = true;
            best_header = candidate_header;
            best_data = candidate_data;
            best_slot = slot;
        }
    }

    if (!found) {
        return false;
    }

    (void)logger_read_snapshot_slot(best_slot, true, NULL, NULL);

    if (header != NULL) {
        *header = best_header;
    }
    if (data != NULL) {
        *data = best_data;
    }
    if (slot_index != NULL) {
        *slot_index = best_slot;
    }

    return true;
}

/* Erase the flash pages that will hold one whole snapshot slot. */
static bool flash_erase_pages(uint32_t addr_start, uint32_t pages)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t page_error = 0u;

    s_last_flash_error = HAL_FLASH_ERROR_NONE;
    HAL_FLASH_Unlock();

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page      = (addr_start - FLASH_BASE) / LOGGER_FLASH_PAGE_SIZE;
    erase_init.NbPages   = pages;
#if defined(FLASH_BANK_1)
    erase_init.Banks     = FLASH_BANK_1;
#endif

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    HAL_FLASH_Lock();
    if (status != HAL_OK) {
        s_last_flash_error = HAL_FLASH_GetError();
    }
    return (status == HAL_OK);
}

/* Program an arbitrary byte stream into flash using aligned doubleword writes. */
static bool flash_program_bytes(uint32_t dst, const uint8_t *src, uint32_t len)
{
    HAL_StatusTypeDef status;

    s_last_flash_error = HAL_FLASH_ERROR_NONE;
    HAL_FLASH_Unlock();

    while (len != 0u) {
        uint64_t doubleword = 0xFFFFFFFFFFFFFFFFull;
        uint32_t chunk = (len >= 8u) ? 8u : len;

        memcpy(&doubleword, src, chunk);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dst, doubleword);
        if (status != HAL_OK) {
            s_last_flash_error = HAL_FLASH_GetError();
            HAL_FLASH_Lock();
            return false;
        }

        dst += 8u;
        src += chunk;
        len -= chunk;
    }

    HAL_FLASH_Lock();
    return true;
}

/* Persist the complete RAM disk plus metadata to the next rotating flash slot. */
bool logger_persist_ramdisk_to_flash(void)
{
    logger_snapshot_header_t latest_header;
    logger_snapshot_header_t header = {0};
    logger_snapshot_header_t verified_header;
    uint32_t latest_slot = 0u;
    uint32_t target_slot;
    uint32_t target_address;

    header.magic      = LOGGER_SNAPSHOT_MAGIC;
    header.version    = LOGGER_SNAPSHOT_VERSION;
    header.sequence   = 1u;
    header.data_len   = LOGGER_BACKUP_BYTES;
    header.metadata_len = s_snapshot_metadata_len;
    header.log_offset = log_offset;
    header.record_counter = s_record_counter;
    header.flags      = s_wrapped ? LOGGER_SNAPSHOT_FLAG_WRAPPED : 0u;

    if (logger_find_latest_snapshot(&latest_header, NULL, &latest_slot)) {
        header.sequence = latest_header.sequence + 1u;
        target_slot = (latest_slot + 1u) % LOGGER_SNAPSHOT_SLOT_COUNT;
    } else {
        target_slot = 0u;
    }

    header.crc32 = logger_snapshot_crc(&header, s_snapshot_metadata, ram_disk);
    target_address = logger_snapshot_slot_address(target_slot);

    if (!flash_erase_pages(target_address, LOGGER_SNAPSHOT_SLOT_PAGES)) {
        return false;
    }
    if (!flash_program_bytes(target_address, (const uint8_t *)&header, sizeof(header))) {
        return false;
    }
    if (!flash_program_bytes(target_address + LOGGER_SNAPSHOT_HEADER_SIZE, s_snapshot_metadata, LOGGER_SNAPSHOT_METADATA_MAX_BYTES)) {
        return false;
    }
    if (!flash_program_bytes(target_address + LOGGER_SNAPSHOT_DATA_OFFSET, ram_disk, LOGGER_BACKUP_BYTES)) {
        return false;
    }
    if (!logger_read_snapshot_slot(target_slot, false, &verified_header, NULL) ||
        (verified_header.sequence != header.sequence)) {
        s_last_flash_error = LOGGER_FLASH_ERROR_VERIFY;
        return false;
    }

    s_last_flash_error = HAL_FLASH_ERROR_NONE;
    return true;
}

/* Restore the newest valid flash snapshot back into the live RAM disk buffer. */
bool logger_restore_ramdisk_from_flash(void)
{
    logger_snapshot_header_t header;
    const uint8_t *data;

    if (!logger_ramdisk_is_empty()) {
        return false;
    }
    if (!logger_find_latest_snapshot(&header, &data, NULL)) {
        return false;
    }

    memcpy(ram_disk, data, header.data_len);
    log_offset = header.log_offset;
    s_record_counter = header.record_counter;
    s_wrapped = ((header.flags & LOGGER_SNAPSHOT_FLAG_WRAPPED) != 0u);
    return true;
}

/* Quick validity probe used by runtime code before attempting a full restore. */
bool logger_flash_snapshot_is_available(void)
{
    return logger_find_latest_snapshot(NULL, NULL, NULL);
}

/* Detect whether the RAM disk currently contains a plausible FAT boot sector. */
bool logger_ramdisk_is_empty(void)
{
    const uint8_t *boot_sector = &ram_disk[0];
    uint16_t signature = ((uint16_t)boot_sector[511] << 8) | boot_sector[510];
    uint16_t bytes_per_sector = (uint16_t)boot_sector[11] | ((uint16_t)boot_sector[12] << 8);

    if (signature != 0xAA55u) {
        return true;
    }
    if (bytes_per_sector != SECTOR_SIZE) {
        return true;
    }

    return false;
}

/* Deliberately break the FAT signature so runtime knows the live RAM disk is no longer valid. */
void logger_invalidate_ramdisk(void)
{
    ram_disk[510] = 0x00u;
    ram_disk[511] = 0x00u;
}
