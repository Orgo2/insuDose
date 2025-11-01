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
#define enter_Pin GPIO_PIN_0
#define enter_GPIO_Port GPIOA
#define buzz_Pin GPIO_PIN_2
#define buzz_GPIO_Port GPIOB
#define cs_Pin GPIO_PIN_15
#define cs_GPIO_Port GPIOA
#define D_sclk_Pin GPIO_PIN_3
#define D_sclk_GPIO_Port GPIOB
#define dc_Pin GPIO_PIN_4
#define dc_GPIO_Port GPIOB
#define D_mosi_Pin GPIO_PIN_5
#define D_mosi_GPIO_Port GPIOB
#define res_Pin GPIO_PIN_6
#define res_GPIO_Port GPIOB
#define busy_Pin GPIO_PIN_7
#define busy_GPIO_Port GPIOB
#define busy_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
