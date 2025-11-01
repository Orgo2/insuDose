/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Display_EPD_W21_spi.h"
#include "Display_EPD_W21.h"
#include "fonts.h"
#include "GUI_Paint.h"
#include <string.h>
#include "rtc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_PIN GPIO_PIN_0
#define BUTTON_PORT GPIOA
#define PIEZO_PIN GPIO_PIN_2
#define PIEZO_PORT GPIOB
#define Butt_TIMEOUT_MS 2000
#define BOOT_ADD 0x1FFF0000 //0x1FFF0000 stm32wb55 bootloader address
#define BootJumpTime 3000
static uint8_t BlackImage[EPD_WIDTH * EPD_HEIGHT / 8]; //display drawing buffer
///display pins definitions/////

/////end of display pins definitions////
/* USER CODE END PD */
extern RTC_HandleTypeDef hrtc;
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
LPTIM_HandleTypeDef hlptim1;




/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPTIM1_Init(void);

/* USER CODE BEGIN PFP */
void PlayBeep(uint32_t duration_ms);
void JumpToBootloader(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */
  EPD_GPIO_Init();              // Inicializácia GPIO pre displej
  EPD_HW_Init();                // Inicializácia displeja
  MX_RTC_Init();    //inicializacia rtc
  MX_I2C1_Init(); 	//inicializacia I2c
  //5s window for bootloader jump
  uint32_t buttonPressStartTime = 0;
  uint32_t TimeoutCounter = 0;
  uint8_t buttonHeld = 0;
  TimeoutCounter = HAL_GetTick();
  //till boot jump time period is gone, stay in the loop and read the button
  while ( HAL_GetTick() - TimeoutCounter <= BootJumpTime ) {
	  //read the button. if button is zero(pulldown) start the countdown
	  	  if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_RESET) {
	  	             if (!buttonHeld) {
	  	                 buttonHeld = 1;
	  	                 buttonPressStartTime = HAL_GetTick();
	  	             //if we pass 2s debouncing play beep and jump to the bootloader
	  	             } else if (HAL_GetTick() - buttonPressStartTime >= Butt_TIMEOUT_MS) {
	  	                 PlayBeep(1000); // Pípnutie na 1 sekundu
	  	                 JumpToBootloader();
	  	             }
	  	  }
	  	  else {
	  	             buttonHeld = 0; // if button was pressed shorter time release the flag

	  	  }



  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Načítať čas a dátum
	     RTC_TimeTypeDef sTime;
	     RTC_DateTypeDef sDate;
	     char timeStr[16];
	     char dateStr[16];

	     HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	     HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // Dôležité: načítaj Date hneď po Time!

	     // Vytvoriť texty pre výpis
	     sprintf(timeStr, "%02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
	     sprintf(dateStr, "%02d.%02d.20%02d", sDate.Date, sDate.Month, sDate.Year);

	     /////////////////// Vykreslenie na e-paper /////////////////////
	     Paint_NewImage(BlackImage, EPD_WIDTH, EPD_HEIGHT, ROTATE_90, WHITE);
	     Paint_SetMirroring(MIRROR_VERTICAL);
	     Paint_Clear(WHITE); // Vyčisti obraz

	     // Vykresliť dátum
	     Paint_DrawString_EN(5, 5, dateStr, &Font16, WHITE, BLACK);

	     // Vykresliť čas
	     Paint_DrawString_EN(5, 30, timeStr, &Font24, WHITE, BLACK);

	     // Odošli na displej
	     EPD_Display(BlackImage);

	     // Uspi displej pre úsporu
	     EPD_DeepSleep();

	     // Počkaj 5 sekúnd pred ďalšou aktualizáciou
	     HAL_Delay(5000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV4;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(cs_GPIO_Port, cs_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, dc_Pin|res_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : enter_Pin */
  GPIO_InitStruct.Pin = enter_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(enter_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : cs_Pin */
  GPIO_InitStruct.Pin = cs_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(cs_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : dc_Pin res_Pin */
  GPIO_InitStruct.Pin = dc_Pin|res_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : busy_Pin */
  GPIO_InitStruct.Pin = busy_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(busy_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */


/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void PlayBeep(uint32_t duration_ms) {
	uint32_t period = 500;  // 1 kHz (f=clk1MHz/1000)
	uint32_t pulse = 300;   // 50% duty cycle (D=pulse/period)
	// Spusti časovač na generovanie PWM
    HAL_LPTIM_PWM_Start(&hlptim1, period, pulse);

    // Čakaj, kým uplynie trvanie zvuku
    HAL_Delay(duration_ms);

    // Zastav časovač
    HAL_LPTIM_PWM_Stop(&hlptim1);
}

void JumpToBootloader(void) {


    // Vypnutie prerušení
    __disable_irq();

    // Reset USB
    USB->CNTR = 0x0003;

    // Resetovanie periférií
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOA_CLK_DISABLE();
    //USB->CNTR = 0x0003;
    HAL_LPTIM_DeInit(&hlptim1);
    HAL_RCC_DeInit();
   // HAL_DeInit();
    // resetne systivk
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    // Clear all interrupt bits
      for (uint8_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++)
      {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
      }

      /* Re-enable all interrupts */
      __enable_irq();

      /* Set up the jump to boot loader address + 4 */
      uint32_t jump_address = *(__IO uint32_t *)(BOOT_ADD + 4);

      /* Set the main stack pointer to the boot loader stack */
      __set_MSP(*(uint32_t *)BOOT_ADD);

      /* Call the function to jump to boot loader location */
      void (*boot_load)(void) = (void (*)(void))(jump_address);

      //remap memory
      SYSCFG->MEMRMP = 0x01;

      // Now jump to the boot loader
      boot_load();
      /* Jump is done successfully */

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
