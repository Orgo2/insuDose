/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file implements a RAM disk driver for FatFs.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "logger.h"  // provides ram_disk, SECTOR_SIZE, NUM_SECTORS

/* Private variables ---------------------------------------------------------*/
static volatile DSTATUS Stat = 0; /* Disk status: 0 = OK */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  (void)pdrv;
  Stat = 0; /* ready */
  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  (void)pdrv;
  return Stat; /* always ready */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  (void)pdrv;
  if ((sector + count) > NUM_SECTORS) {
    return RES_PARERR;
  }
  memcpy(buff, &ram_disk[sector * SECTOR_SIZE], (size_t)count * SECTOR_SIZE);
  return RES_OK;
}

#if _USE_WRITE == 1
/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  (void)pdrv;
  if ((sector + count) > NUM_SECTORS) {
    return RES_PARERR;
  }
  memcpy(&ram_disk[sector * SECTOR_SIZE], buff, (size_t)count * SECTOR_SIZE);
  return RES_OK;
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  (void)pdrv;
  switch (cmd) {
    case CTRL_SYNC:
      return RES_OK;
    case GET_SECTOR_COUNT:
      *(DWORD*)buff = NUM_SECTORS;
      return RES_OK;
    case GET_SECTOR_SIZE:
      *(WORD*)buff = SECTOR_SIZE;
      return RES_OK;
    case GET_BLOCK_SIZE:
      *(DWORD*)buff = 1; /* erase block size in sectors (not applicable) */
      return RES_OK;
    default:
      return RES_PARERR;
  }
}
#endif /* _USE_IOCTL == 1 */