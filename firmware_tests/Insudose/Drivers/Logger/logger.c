#include "logger.h"
#include "stm32wbxx_hal_flash.h"
#include "stm32wbxx_hal_flash_ex.h"

static volatile bool s_usb_active = false;
void logger_set_usb_active(bool active) { s_usb_active = active; }
// keď raz dôjde k wrapu(naplni sa pamat), ostávame v „ring" režime
static bool s_wrapped = false;


uint32_t log_offset = 0; //definuje ukazovatel na roundrobin
extern void PlayBeep(uint32_t duration_ms);
uint8_t ram_disk[SECTOR_SIZE * NUM_SECTORS] = {0};


//////////////init ramdisk/////////////////
bool init_ramdisk(void) {

    FRESULT res = f_mount(&USERFatFs, "0:", 1);
    if (res == FR_NO_FILESYSTEM) {
        static BYTE work[512];

        res = f_mkfs("0:", FM_FAT | FM_SFD, 512, work, sizeof(work)); //funkcna fat12


        if (res != FR_OK) {

        	return false;
        }

        res = f_mount(&USERFatFs, "0:", 1); // remount
        if (res != FR_OK){

        }
    }

    return (res == FR_OK);
}
///////////end of ramdisk init func/////////////////////

/////////roundrobin write data//////////////
void append_log_rotating(uint16_t year, uint8_t mon, uint8_t day,
                         uint8_t hour, uint8_t min,
                         uint8_t davka, int8_t teplota_c) {
    // počas MSC nepíšeme
    if (s_usb_active) return;

    FIL file;
    FRESULT res;
    UINT bw;
    char line[MAX_LOG_LINE_LEN];

    // --- zostavenie riadku s podporou "lomítok" ---
    // čas: ak ktorýkoľvek komponent je sentinel, zapíšeme 12 lomítok
    bool time_missing = (year == LOG_NO_YEAR) || (mon == LOG_NO_U8) ||
                        (day  == LOG_NO_U8)   || (hour== LOG_NO_U8) ||
                        (min  == LOG_NO_U8);

    char ts[13]; // 12 znakov + '\0'
    if (!time_missing) {
    	// nechaj: char ts[13]; toto je aby kompilator vedel ze ziaden znak neprekroci danu hodnotu
    	snprintf(ts, sizeof(ts), "%04u%02u%02u%02u%02u",
		         (unsigned)(year % 10000u),
		         (unsigned)(mon  % 100u),
		         (unsigned)(day  % 100u),
		         (unsigned)(hour % 100u),
		         (unsigned)(min  % 100u));
    } else {
        memset(ts, '/', 12);
        ts[12] = '\0';
    }

    // dávka: ak sentinel → "/"
    char dosebuf[5];
    if (davka != LOG_NO_U8) snprintf(dosebuf, sizeof(dosebuf), "%u", davka);
    else strcpy(dosebuf, "/");

    // teplota: ak sentinel → "/"
    char tempbuf[6];
    if (teplota_c != LOG_NO_TEMP) snprintf(tempbuf, sizeof(tempbuf), "%d", teplota_c);
    else strcpy(tempbuf, "/");

    // výsledný riadok: "YYYYMMDDHHMM,davka,teplota\r\n" alebo "////////////,/ ,/\r\n"
    snprintf(line, sizeof(line), "%s,%s,%s\r\n", ts, dosebuf, tempbuf);
    UINT line_len = (UINT)strlen(line);

    // --- otvor súbor ---
    res = f_open(&file, LOG_FILE_NAME, FA_WRITE | FA_OPEN_ALWAYS);
    if (res != FR_OK) return;

    FSIZE_t size = f_size(&file);

    if (!s_wrapped && size < LOG_FILE_SIZE) {
        // FÁZA 1: prirodzený rast len reálnymi dátami
        log_offset = (uint32_t)size;

        if ((size + line_len) <= LOG_FILE_SIZE) {
            // ešte sa zmestí → append na koniec
            f_lseek(&file, size);
            f_write(&file, line, line_len, &bw);
            log_offset += line_len;
        } else {
            // týmto riadkom by sme presiahli limit → prechod do ring režimu
            s_wrapped = true;
            log_offset = 0;
            f_lseek(&file, 0);
            f_write(&file, line, line_len, &bw);
            log_offset = line_len;
            // veľkosť ostáva na doterajšom "high-watermark"; nič nepredvypĺňame
        }
    } else {
        // FÁZA 2: ring režim (prepis najstarších dát)
        if (log_offset > (LOG_FILE_SIZE - MAX_LOG_LINE_LEN)) {
            log_offset = 0; // neštiepiť riadok cez koniec
        }
        f_lseek(&file, log_offset);
        f_write(&file, line, line_len, &bw);

        log_offset += line_len;
        if (log_offset > (LOG_FILE_SIZE - MAX_LOG_LINE_LEN)) {
            log_offset = 0;
        }
    }

    f_close(&file);
}
//////ram-flash uvlo backup///

// --- jednoduchý CRC32 (polynóm 0x04C11DB7) ---
static uint32_t crc32_update(uint32_t crc, const uint8_t *p, uint32_t len)
{
  crc = ~crc;
  for (uint32_t i = 0; i < len; i++) {
      uint32_t c = p[i];
      crc ^= c;
      for (uint32_t b = 0; b < 8; b++) {
          uint32_t mask = -(crc & 1u);
          crc = (crc >> 1) ^ (0xEDB88320u & mask);
      }
  }
  return ~crc;
}

typedef struct {
  uint32_t magic;       // 'RDS1'
  uint32_t version;     // 1
  uint32_t data_len;    // = LOGGER_BACKUP_BYTES
  uint32_t crc32;       // crc z ram_disk[0..data_len-1]
  uint32_t reserved[4]; // zarovnanie na 32 bajtov
} rd_hdr_t;

static uint32_t find_erase_block_start(uint32_t pages_needed)
{
  // Hľadáme súvislý blok "pages_needed" **vymazaných** strán od konca FLASH dozadu
  uint32_t flash_start = FLASH_BASE;
  uint32_t flash_end   = LOGGER_FLASH_END;
  uint32_t total_pages = (flash_end - flash_start) / LOGGER_FLASH_PAGE_SIZE;

  int32_t run = 0;
  for (int32_t p = (int32_t)total_pages - 1; p >= 0; p--) {
      uint32_t addr = flash_start + (uint32_t)p * LOGGER_FLASH_PAGE_SIZE;
      // stránka je považovaná za voľnú, ak prvé slovo je 0xFFFFFFFF
      if (*(volatile uint32_t *)addr == 0xFFFFFFFFu) {
          run++;
          if ((uint32_t)run >= pages_needed) {
              int32_t first = p;
              return flash_start + (uint32_t)first * LOGGER_FLASH_PAGE_SIZE;
          }
      } else {
          run = 0;
      }
  }
  return 0u; // nenašli sme súvislý voľný blok
}

static bool flash_erase_pages(uint32_t addr_start, uint32_t pages)
{
  HAL_StatusTypeDef st;
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef ei = {0};
  ei.TypeErase = FLASH_TYPEERASE_PAGES;
  ei.Page      = (addr_start - FLASH_BASE) / LOGGER_FLASH_PAGE_SIZE;
  ei.NbPages   = pages;
#if defined(FLASH_BANK_1)
  ei.Banks     = FLASH_BANK_1;
#endif

  uint32_t page_err = 0;
  st = HAL_FLASHEx_Erase(&ei, &page_err);
  HAL_FLASH_Lock();
  return (st == HAL_OK);
}

static bool flash_program_bytes(uint32_t dst, const uint8_t *src, uint32_t len)
{
  HAL_StatusTypeDef st;
  HAL_FLASH_Unlock();

  // WB flash programuje 64-bit "doubleword"
  while (len) {
      uint64_t dq = 0xFFFFFFFFFFFFFFFFull;
      uint32_t chunk = (len >= 8u) ? 8u : len;
      memcpy(&dq, src, chunk);
      st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dst, dq);
      if (st != HAL_OK) { HAL_FLASH_Lock(); return false; }
      dst += 8u;
      src += chunk;
      len -= chunk;
  }

  HAL_FLASH_Lock();
  return true;
}

bool logger_persist_ramdisk_to_flash(void)
{
  rd_hdr_t hdr;
  hdr.magic    = 0x31534452u; // 'RDS1' little-endian
  hdr.version  = 1u;
  hdr.data_len = LOGGER_BACKUP_BYTES;
  hdr.crc32    = crc32_update(0u, ram_disk, LOGGER_BACKUP_BYTES);

  // 1) nájdi voľný blok na konci FLASH
  uint32_t pages_needed = LOGGER_BACKUP_PAGES;
  uint32_t start = find_erase_block_start(pages_needed);
  if (start == 0u) {
      // nie je voľné miesto → neriskuj, skonči
      return false;
  }

  // 2) vymaž stránky
  if (!flash_erase_pages(start, pages_needed)) return false;

  // 3) zapíš header + dáta
  uint32_t addr = start;
  if (!flash_program_bytes(addr, (const uint8_t*)&hdr, sizeof(hdr))) return false;
  addr += ((sizeof(hdr) + 7u) & ~7u);
  if (!flash_program_bytes(addr, ram_disk, LOGGER_BACKUP_BYTES)) return false;

  return true;
}

bool logger_restore_ramdisk_from_flash(void)
{
	// Ak má RAM disk platnú FAT, nenechaj ho prepísať
	  if (!logger_ramdisk_is_empty()) {
	    return false;
	  }
  // hľadáme najnovší snapshot: najprv od konca FLASH nájdeme platný header
  uint32_t flash_start = FLASH_BASE;
  uint32_t flash_end   = LOGGER_FLASH_END;
  uint32_t total_pages = (flash_end - flash_start) / LOGGER_FLASH_PAGE_SIZE;
  uint32_t pages_needed = LOGGER_BACKUP_PAGES;

  for (int32_t p = (int32_t)total_pages - (int32_t)pages_needed; p >= 0; p--) {
      uint32_t addr = flash_start + (uint32_t)p * LOGGER_FLASH_PAGE_SIZE;
      rd_hdr_t *h = (rd_hdr_t *)addr;
      if (h->magic == 0x31534452u && h->version == 1u && h->data_len == LOGGER_BACKUP_BYTES) {
          uint32_t hdr_size = ((sizeof(rd_hdr_t) + 7u) & ~7u);
          const uint8_t *data = (const uint8_t *)(addr + hdr_size);
          uint32_t crc = crc32_update(0u, data, h->data_len);
          if (crc == h->crc32) {
              memcpy(ram_disk, data, h->data_len);
              return true;
          }
      }
  }
  return false;
}
bool logger_ramdisk_is_empty(void)
{
    const uint8_t *b0 = &ram_disk[0];
    uint16_t sig = ((uint16_t)b0[511] << 8) | b0[510];  // FAT boot signature
    uint16_t bps = (uint16_t)b0[11] | ((uint16_t)b0[12] << 8); // bytes/sector
    if (sig != 0xAA55) return true;
    if (bps != 512)    return true;
    return false;
}

void logger_invalidate_ramdisk(void)
{
    // zneplatni FAT podpis → pri ďalšom boote/USB vieme, že je „prázdny“
    ram_disk[510] = 0x00;
    ram_disk[511] = 0x00;
}