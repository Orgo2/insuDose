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
#include "app_fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "logger.h"
#include "usbd_core.h"
#include "usbd_def.h"
#include "charger.h"
#include "ff.h"         // kvôli f_mount
#include "stm32wbxx_hal.h"

extern USBD_HandleTypeDef hUsbDeviceFS;  // z usb_device.c
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
ADC_HandleTypeDef hadc1;

LPTIM_HandleTypeDef hlptim1;

/* USER CODE BEGIN PV */
typedef enum {
  SYS_RUNNING,      // round-robin beží, USB vypnuté
  SYS_USB_SWITCH,   // prechod po zmene VBUS (debounce + prepínač)
  SYS_USB_ACTIVE    // MSC aktívne, round-robin stojí
} sys_state_t;

static volatile sys_state_t g_state = SYS_RUNNING;
static volatile uint8_t  g_vbus_present = 0;
static volatile uint32_t g_vbus_last_change_ms = 0;
static uint8_t usb_started = 0;
static volatile uint8_t dose_irq = 0;
static volatile uint8_t beep_request = 0; //test button

// informujeme logger, či je USB aktívne (aby počas MSC nezapisoval)
extern void logger_set_usb_active(bool active);
//usb flag na sleepmode
static volatile uint8_t s_usb_active = 0;
//low battery measurement flag
static volatile uint8_t g_wake_lowbatt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

void PlayBeep(uint32_t duration_ms);

// Stavový automat + helpery
static void App_StateMachine_Tick(void);
static void Buttons_Tick(void);
static void rr_start(void);		///zapne roundrobin(nenapisane)
static void rr_stop(void);
static void usb_enter(void);	//funkcia na usb inicializaciu
static void usb_leave(void);	//funkcia usb deinit
static void power_set_usb_active(bool on);	//low power mod na usb
static void power_sleep_until_event(void);	//sleepmod stop2
static void PVD_Init(void);		//detekcia slabej baterky
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
  //__HAL_RCC_SYSCFG_CLK_ENABLE();
  MX_GPIO_Init();
  //MX_USB_Device_Init();
  MX_LPTIM1_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //PlayBeep(500);

  charger_init(false);	//vypne nabijanie aby bolo mozne pracovat s nenabijacou bateriou

  // RAM disk: obnov len ak je naozaj prázdny, inak len pripoj
  if (logger_ramdisk_is_empty()) {
      if (logger_restore_ramdisk_from_flash()) {
          (void)f_mount(&USERFatFs, "0:", 1);
      } else {
          init_ramdisk();
      }
  } else {
      (void)f_mount(&USERFatFs, "0:", 1);
  }

  PVD_Init();

  // Inicializovať usbMSC alebo datalogger podľa VBUS (PE4: 1 = pripojené USB)
  g_vbus_present = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) ? 1U : 0U;
  g_vbus_last_change_ms = HAL_GetTick();

  if (g_vbus_present) {
    rr_stop();
    usb_enter();                   // spustí USB MSC
    logger_set_usb_active(true);   // logger nebude zapisovať
    g_state = SYS_USB_ACTIVE;
  } else {
    logger_set_usb_active(false);
    rr_start();
    g_state = SYS_RUNNING;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (beep_request) { beep_request = 0; PlayBeep(40); } //testovanie prerusenia

	  // --- LOW BATTERY PATH ---
	  if (g_wake_lowbatt) {
	    g_wake_lowbatt = 0;

	    int32_t mv = charger_read_bat();   // vráti -1 ak CHARGER_USE_ADC=0
	    if (mv < 0 || mv < 2800) {
	      // Ulož RAM disk do FLASH a zneplatni RAM FS, aby sa po VBUS obnovil zo snapshotu
	      if (logger_persist_ramdisk_to_flash()) {
	        logger_invalidate_ramdisk();
	      }

	      // Prejdi do úsporného režimu a čakaj na udalosť (napr. pripojenie USB)

	      power_sleep_until_event();
	      continue;  // preskoč zvyšok iterácie
	    }
	  }
	  ////po stlaceni tlacidla dose nahra jeden riadok kodu
	  App_StateMachine_Tick();
	   Buttons_Tick();
	   power_sleep_until_event();   // namiesto HAL_Delay(10)
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV32;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
 __HAL_RCC_GPIOE_CLK_ENABLE();  // kvôli PE4 (VBUS sense)
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // PA2 = DOSE (všetky tlačidlá proti GND → potrebuje PULLUP)
  GPIO_InitStruct.Pin  = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PE4 = VBUS sense, EXTI na obe hrany
  GPIO_InitStruct.Pin  = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // NVIC pre EXTI4
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  // NVIC pre EXTI2 (PA2)
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/////////////////piezo function/////////////////
void PlayBeep(uint32_t duration_ms) {
	uint32_t period = 15;  // 4 kHz (f=clk 4MHz/999) alebo 32khz/8
	uint32_t pulse = 8;   // 50% duty cycle (D=pulse/period)
	// Spusti časovač na generovanie PWM
    HAL_LPTIM_PWM_Start(&hlptim1, period, pulse);

    // Čakaj, kým uplynie trvanie zvuku
    HAL_Delay(duration_ms);

    // Zastav časovač
    HAL_LPTIM_PWM_Stop(&hlptim1);
}

// --- VBUS EXTI callback s debounce (~20 ms) ---
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_4) { // PE4
    uint32_t now = HAL_GetTick();
    if ((now - g_vbus_last_change_ms) < 20) return; // debounce
    g_vbus_last_change_ms = now;

    g_vbus_present = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) ? 1U : 0U;
    g_state = SYS_USB_SWITCH;
  }
  if (GPIO_Pin == GPIO_PIN_2) {   // DOSE
    dose_irq = 1; beep_request = 1;
    return;
   }

}

// --- Stavový automat: prepína RUNNING <-> USB_ACTIVE ---
static void App_StateMachine_Tick(void)
{
  switch (g_state) {
    case SYS_RUNNING:
      // bežná prevádzka, logovanie povolené
      break;

    case SYS_USB_SWITCH:
      if (g_vbus_present) {
        rr_stop();
        usb_enter();
        logger_set_usb_active(true);
        g_state = SYS_USB_ACTIVE;
      } else {
        usb_leave();
        logger_set_usb_active(false);
        rr_start();
        g_state = SYS_RUNNING;
      }
      break;

    case SYS_USB_ACTIVE:
      // MSC beží, logovanie je pozastavené (logger to vie z flagu)
      break;
  }
}

// --- Čítanie DOSE (PA2). Stlačenie = jeden záznam (1u, fixný čas/teplota) ---
static void Buttons_Tick(void)
{
  if (!dose_irq) return;
  dose_irq = 0;

  if (g_state == SYS_RUNNING) {
    append_log_rotating(LOG_NO_YEAR, LOG_NO_U8, LOG_NO_U8, LOG_NO_U8, LOG_NO_U8,
                        1, LOG_NO_TEMP);
    PlayBeep(40);
  }
  // (voliteľné) ak chceš pípnuť aj pri USB:
   else { PlayBeep(40); }
}

// --- Hooky: USB start/stop & RR start/stop ---
static void usb_enter(void)
{
  if (!usb_started) {
    // ak je RAM disk prázdny, skús ho obnoviť zo snapshotu
    if (logger_ramdisk_is_empty()) {
      (void)logger_restore_ramdisk_from_flash();
      // (voliteľné) (void)f_mount(&USERFatFs, "0:", 1);
    }
    MX_USB_Device_Init();
    usb_started = 1;
    power_set_usb_active(true);
  }
}

static void usb_leave(void)
{
  if (usb_started) {
    USBD_Stop(&hUsbDeviceFS);
    usb_started = 0;
    power_set_usb_active(false);
  }
}

static void rr_start(void)
{
  // TODO: sem pridaj spúšťanie tvojho round-robin (časovače/refresh displeja...)
}

static void rr_stop(void)
{
  // TODO: sem pridaj zastavenie round-robin + prípadné uloženie UI stavu
}
	//usb aktivny, nastav1 stavovy automat??
static void power_set_usb_active(bool on) { s_usb_active = on ? 1 : 0; }
	//funkcia na nastavovanie sleepmodov///

static void power_sleep_until_event(void)
{
  if (s_usb_active) {            // keď je USB pripojené, stačí ľahký spánok
    __WFI();
    return;
  }
  HAL_SuspendTick(); //stop systic to prevent cpu wakeup
  HAL_PWREx_EnterSTOP2Mode(PWR_SLEEPENTRY_WFI);
  HAL_ResumeTick();
 // SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;              // STOP2
  //HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);    // wake: EXTI (DOSE/OVERTEMP/VBUS...)
  //SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
  //SystemClock_Config();                           // obnov hodiny po STOP2
  //__WFI();  //pe pripad ze zalomitkujeme ostatne mozeme skusit toto
  return;
}
//kod na detekciu slabej baterie//
static void PVD_Init(void)
{
	return; // DOČASNE: vypni PVD/undervoltage. (Zrušíš zmazaním tohto riadka.)
	  // ... zvyšok pôvodnej PVD_Init() nechaj nedotknutý
  PWR_PVDTypeDef cfg = {0};
  cfg.PVDLevel = PWR_PVDLEVEL_6;
  cfg.Mode     = PWR_PVD_MODE_IT_RISING_FALLING;
  HAL_PWR_ConfigPVD(&cfg);
  HAL_PWR_EnablePVD();

  HAL_NVIC_SetPriority(PVD_PVM_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);
}


// Callback (HAL volá jeden z nich; necháme oba, nech to skompiluje na WB bez .ioc zásahu):
void HAL_PWR_PVDCallback(void)
{
  g_wake_lowbatt = 1;
}

void HAL_PWREx_PVD_PVMCallback(PWR_PVMTypeDef PVD_PVM)
{
  (void)PVD_PVM;
  g_wake_lowbatt = 1;
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
