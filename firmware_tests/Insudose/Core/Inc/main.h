/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"

#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENT_Pin GPIO_PIN_0
#define ENT_GPIO_Port GPIOA
#define ENT_EXTI_IRQn EXTI0_IRQn
#define BATT_Pin GPIO_PIN_1
#define BATT_GPIO_Port GPIOA
#define DOSE_Pin GPIO_PIN_2
#define DOSE_GPIO_Port GPIOA
#define DOSE_EXTI_IRQn EXTI2_IRQn
#define PW_TMP_Pin GPIO_PIN_4
#define PW_TMP_GPIO_Port GPIOA
#define ESC_Pin GPIO_PIN_5
#define ESC_GPIO_Port GPIOA
#define LIST_Pin GPIO_PIN_6
#define LIST_GPIO_Port GPIOA
#define PW_MIC_Pin GPIO_PIN_7
#define PW_MIC_GPIO_Port GPIOA
#define BUZZ_Pin GPIO_PIN_2
#define BUZZ_GPIO_Port GPIOB
#define CEN_Pin GPIO_PIN_0
#define CEN_GPIO_Port GPIOB
#define USB_I_Pin GPIO_PIN_4
#define USB_I_GPIO_Port GPIOE
#define USB_I_EXTI_IRQn EXTI4_IRQn
#define D_CS_Pin GPIO_PIN_15
#define D_CS_GPIO_Port GPIOA
#define D_SCLK_Pin GPIO_PIN_3
#define D_SCLK_GPIO_Port GPIOB
#define D_DC_Pin GPIO_PIN_4
#define D_DC_GPIO_Port GPIOB
#define D_MOSI_Pin GPIO_PIN_5
#define D_MOSI_GPIO_Port GPIOB
#define D_RST_Pin GPIO_PIN_6
#define D_RST_GPIO_Port GPIOB
#define D_BUSY_Pin GPIO_PIN_7
#define D_BUSY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
