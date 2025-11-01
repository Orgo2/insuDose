/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define AUDIO_BUF 64 //amount of buffer samples
int i = 0;
int32_t RecBuff [AUDIO_BUF]; //pdm data buffer
int16_t PlayBuff1 [AUDIO_BUF/2];//PCM data buffer
int16_t PlayBuff2 [AUDIO_BUF/2];//PCM data buffer
uint32_t DmaRecHalfBuffCplt =0;//mic halfdata sent
uint32_t DmaRecBuffCplt = 0;//mic transfer finished
int16_t	minthreshold = 400;
int16_t	maxthreshold = 3000;
//int16_t trigger = 32767;
int16_t test1 = 0;
uint8_t davka = 0;
uint8_t Test[] = "jednotiek\r\n";
///vyhladavanie pola dlzok///
uint32_t simbuf1 = 0; //simple 32bit buffer
uint32_t simbuf2 = 0; //simple 32bit buffer
unsigned int count = 0;
uint32_t pattern = 0b01110;  // Binárny vzorec, ktorý hľadáme (01110)
unsigned int pattern_length = 5; // Dĺžka vzorca (5 bitov)





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//funkcia na posielanie dat cez uart
//void UART_DMA_Transmit(int16_t* pBuffer, uint16_t size)
//{
    // Čakáme na dokončenie predchádzajúceho prenosu
  //  if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
   // {
  //      return;  // UART nie je pripravený, môže byť ešte aktívny iný prenos
  //  }

    // Spustenie prenosu cez DMA
  //  if (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)pBuffer, size * sizeof(int16_t)) != HAL_OK)
  //  {
  //      Error_Handler(); // Funkcia na spracovanie chýb
  //  }
//}
///////funkcia hlada cvaknutia v case///
unsigned int count_pattern(uint32_t simbuf) {

    // Prejdeme celé číslo 'a' a porovnáme každý 5-bitový segment
    for (int i = 0; i <= 27; i++) {  // Posúvame vzorec po celom čísle (od 0 do 27)
        uint32_t shifted = simbuf >> i;  // Posuneme číslo o 'i' bitov doprava
        if ((shifted & 0b11111) == pattern) {  // Skontrolujeme, či sa 5-bitový vzorec zhoduje
            count++;
        }
    }

    return count;
}
// Funkcia na hladanie cvakov v amplitude - porovnáva číslo a ak je v požadovanom rozsahu, pripočíta jeden bit do uint32_t
void check_and_increment(int number, uint32_t *counter) {
    // Kontrola, či číslo je v rozsahu -3000 až -400 alebo 400 až 3000
    if ((number >= -maxthreshold && number <= -minthreshold) || (number >= minthreshold && number <= maxthreshold)) {
        // Ak je v rozsahu, inkrementujeme premennej counter
        (*counter)++;
    }
}

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DFSDM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_DFSDM_FilterRegularStart_DMA (&hdfsdm1_filter0, (int32_t*) RecBuff, AUDIO_BUF);
  //HAL_UART_Transmit_DMA(&huart1, (uint8_t*)PlayBuff1, sizeof(PlayBuff1));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	 		if (DmaRecHalfBuffCplt == 1) //processing of the first half of the buffer
	 		{
	 	  /*
	 	  Store values on Play buff */
	 		  for( i = 0; i < AUDIO_BUF/2; i++){
	 			 PlayBuff1[i] = RecBuff [i] >> 16;//example of PCM data postprocessing
	 			 	 // zavoláme funkciu na zistenie amplitudovej podmienky pre každé číslo
	 			 check_and_increment(PlayBuff1[i], &simbuf1);

	 			 //ak je znak z mikrofonu rovny triggru zniz ho o 1
	 			 //if (PlayBuff1[i]>=trigger)
	 		  	 //{
	 		  	//	 PlayBuff1[i]=32766;
	 		  	 //}
	 			 DmaRecHalfBuffCplt  = 0;
	 		  	//HAL_UART_Transmit_DMA(&huart1, (uint8_t*)PlayBuff[i], sizeof(int16_t));
	 		  }
	 		 // Spustenie prenosu cez DMA pre prvú polovicu bufferu
	 		  //PlayBuff1[0]=trigger;
	 		  //zavolame funkciu na zistenie podmienky v case
	 		  davka = count_pattern(simbuf1);
	 		  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)PlayBuff1, (AUDIO_BUF/2) * sizeof(int16_t));
	 		}
	 		if (DmaRecBuffCplt == 1)//processing of the second half of the buffer
	 		{
	 			/* Store values on Play buff */
	 		  for(  i = AUDIO_BUF/2; i < AUDIO_BUF; i++){
	 			  PlayBuff2[i] = RecBuff [i] >> 16;//example of PCM data postprocessing
	 			  //zavolame funkciu na zistenie amplit. podmienky
	 			  check_and_increment(PlayBuff2[i], &simbuf2);
	 			  //test1=PlayBuff2[i];
	 			  //ak sa data z mikrofonu=trigger zniz o 1 (false trig protection)
	 			  //if (PlayBuff2[i]>=trigger)
	 			  //{
	 				//  PlayBuff2[i]=32766;
	 			  //}
	 			  DmaRecBuffCplt  = 0;
	 			  ///prenos dat pre druhu polovicu
	 			  //HAL_UART_Transmit_DMA(&huart1, (uint8_t*)PlayBuff[i], sizeof(int16_t));
	 		  }
	 		 davka = davka + (count_pattern(simbuf2));
	 		  //uint32_t a = 0b11110111011101011110; // Príklad čísla
	 		   // printf("Počet výskytov vzorca 01110: %u\n", count_pattern(a));
	 		    //return 0;
	 		 HAL_UART_Transmit(&huart1, (uint8_t*)&davka, sizeof(davka),10);
	 		 HAL_UART_Transmit(&huart1, (uint8_t*)&Test,  sizeof(Test),10);
	 		 HAL_Delay(10000);
	}

//	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 40;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 40;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 4;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x0;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }


  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : triger_Pin */
  GPIO_InitStruct.Pin = triger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(triger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void
HAL_DFSDM_FilterRegConvHalfCpltCallback (DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
DmaRecHalfBuffCplt = 1;
}
void
HAL_DFSDM_FilterRegConvCpltCallback (DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
DmaRecBuffCplt = 1;
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
