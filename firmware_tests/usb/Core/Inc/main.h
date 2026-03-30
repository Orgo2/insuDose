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
#define Temp_SDA_Pin GPIO_PIN_8
#define Temp_SDA_GPIO_Port GPIOB
#define Temp_SDAB9_Pin GPIO_PIN_9
#define Temp_SDAB9_GPIO_Port GPIOB
#define Enter_Pin GPIO_PIN_0
#define Enter_GPIO_Port GPIOA
#define Enter_EXTI_IRQn EXTI0_IRQn
#define Dose_Pin GPIO_PIN_2
#define Dose_GPIO_Port GPIOA
#define Dose_EXTI_IRQn EXTI2_IRQn
#define Temp_alarm_Pin GPIO_PIN_3
#define Temp_alarm_GPIO_Port GPIOA
#define Temp_alarm_EXTI_IRQn EXTI3_IRQn
#define Temp_PWR_Pin GPIO_PIN_4
#define Temp_PWR_GPIO_Port GPIOA
#define Esc_Pin GPIO_PIN_5
#define Esc_GPIO_Port GPIOA
#define List_Pin GPIO_PIN_6
#define List_GPIO_Port GPIOA
#define M_pwr_Pin GPIO_PIN_7
#define M_pwr_GPIO_Port GPIOA
#define M_ck2_Pin GPIO_PIN_8
#define M_ck2_GPIO_Port GPIOA
#define M_d2_Pin GPIO_PIN_9
#define M_d2_GPIO_Port GPIOA
#define CH_cen_Pin GPIO_PIN_0
#define CH_cen_GPIO_Port GPIOB
#define PWR_detect_Pin GPIO_PIN_4
#define PWR_detect_GPIO_Port GPIOE
#define PWR_detect_EXTI_IRQn EXTI4_IRQn
#define M_d1_chgstate_Pin GPIO_PIN_10
#define M_d1_chgstate_GPIO_Port GPIOA
#define D_cs_Pin GPIO_PIN_15
#define D_cs_GPIO_Port GPIOA
#define D_clk_Pin GPIO_PIN_3
#define D_clk_GPIO_Port GPIOB
#define D_dc_Pin GPIO_PIN_4
#define D_dc_GPIO_Port GPIOB
#define D_mosi_Pin GPIO_PIN_5
#define D_mosi_GPIO_Port GPIOB
#define D_rst_Pin GPIO_PIN_6
#define D_rst_GPIO_Port GPIOB
#define D_busy_Pin GPIO_PIN_7
#define D_busy_GPIO_Port GPIOB
#define D_busy_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
#define Power_Detect_Pin PWR_detect_Pin
#define Power_Detect_GPIO_Port PWR_detect_GPIO_Port
#define Mic_PWR_Pin M_pwr_Pin
#define Mic_PWR_GPIO_Port M_pwr_GPIO_Port
#define Dose_Pin GPIO_PIN_2
#define Dose_GPIO_Port GPIOA
#define Temp_Alert_Pin GPIO_PIN_3
#define Temp_Alert_GPIO_Port GPIOA

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
