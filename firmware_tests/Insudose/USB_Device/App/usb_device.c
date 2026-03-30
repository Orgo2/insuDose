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
static uint8_t s_usb_started = 0;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

extern void Error_Handler(void);

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_Device_Init(void)
{
  /* USER CODE BEGIN USB_Device_Init_PreTreatment */

  /* USER CODE END USB_Device_Init_PreTreatment */

  if (!s_usb_initialized) {
    /* Init Device Library and add supported class.
       Start is done on-demand via USB_Device_Start(). */
    if (USBD_Init(&hUsbDeviceFS, &MSC_Desc, DEVICE_FS) != USBD_OK) {
      Error_Handler();
    }
    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_MSC) != USBD_OK) {
      Error_Handler();
    }
    if (USBD_MSC_RegisterStorage(&hUsbDeviceFS, &USBD_Storage_Interface_fops_FS) != USBD_OK) {
      Error_Handler();
    }
    s_usb_initialized = 1;
  }
  /* USER CODE BEGIN USB_Device_Init_PostTreatment */

  /* USER CODE END USB_Device_Init_PostTreatment */
}

void USB_Device_Start(void)
{
  if (!s_usb_initialized) {
    MX_USB_Device_Init();
  }
  if (!s_usb_started) {
    /* Ensure USB transceiver supply is enabled before connect. */
    HAL_PWREx_EnableVddUSB();

    if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
      Error_Handler();
    }

    /* Diagnostic reconnect sequence: force D+ pull-up re-attach. */
    CLEAR_BIT(USB->BCDR, USB_BCDR_DPPU);
    HAL_Delay(10);
    SET_BIT(USB->BCDR, USB_BCDR_DPPU);

    s_usb_started = 1;
  }
}

void USB_Device_Stop(void)
{
  if (s_usb_initialized && s_usb_started) {
    if (USBD_Stop(&hUsbDeviceFS) != USBD_OK) {
      Error_Handler();
    }
    s_usb_started = 0;
  }
}

/**
  * @}
  */

/**
  * @}
  */

