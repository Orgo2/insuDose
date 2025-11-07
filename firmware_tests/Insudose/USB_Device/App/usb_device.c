/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v3.0_Cube
  * @brief          : This file implements the USB Device
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

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_msc.h"
#include "usbd_storage_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef MSC_Desc;

static uint8_t s_usb_initialized = 0;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

extern void Error_Handler(void);

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_Device_Init(void)
{
  /* USER CODE BEGIN USB_Device_Init_PreTreatment */

  /* USER CODE END USB_Device_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &MSC_Desc, DEVICE_FS) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_MSC) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_MSC_RegisterStorage(&hUsbDeviceFS, &USBD_Storage_Interface_fops_FS) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
    Error_Handler();
  }
  s_usb_initialized = 1;
  /* USER CODE BEGIN USB_Device_Init_PostTreatment */

  /* USER CODE END USB_Device_Init_PostTreatment */
}

/* USER CODE BEGIN FD */
void USB_Device_Start(void)
{
  if (!s_usb_initialized) {
    MX_USB_Device_Init();
    return;
  }
  (void)USBD_Start(&hUsbDeviceFS);
}

void USB_Device_Stop(void)
{
  if (!s_usb_initialized) return;
  (void)USBD_Stop(&hUsbDeviceFS);
}
/* USER CODE END FD */

/**
  * @}
  */

/**
  * @}
  */