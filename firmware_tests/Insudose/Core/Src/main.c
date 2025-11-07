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
#include "adc.h"
#include "app_fatfs.h"
#include "i2c.h"
#include "ipcc.h"
#include "lptim.h"
#include "rf.h"
#include "rtc.h"
#include "sai.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Display_EPD_W21_spi.h"
#include "Display_EPD_W21.h"
#include "fonts.h"
#include "GUI_Paint.h"
#include <string.h>
#include "stdio.h"
#include "tmp102.h"
#include "logger.h"
#include "charger.h"
#define ENABLE_EPD 0
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUTTON_PIN GPIO_PIN_0
#define BUTTON_PORT GPIOA
#define PIEZO_PIN GPIO_PIN_2
#define PIEZO_PORT GPIOB
#define Butt_TIMEOUT_MS 2000
#define BOOT_ADD 0x1FFF0000 //0x1FFF0000 stm32wb55 bootloader address
#define BootJumpTime 3000

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern RTC_HandleTypeDef hrtc;
//LPTIM_HandleTypeDef hlptim1;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//static uint8_t BlackImage[EPD_WIDTH * EPD_HEIGHT / 8]; //display drawing buffer
static uint8_t BlackImage[EPD_WIDTH * EPD_HEIGHT / 8];
static uint8_t epd_inited = 0;
uint8_t dose = 0;
float temperature = 0.0f;
static volatile uint8_t dose_event_pending = 0;
static volatile uint8_t usb_event_pending = 0; // edge detected on USB 5V pin
static uint8_t usb_connected = 0; // tracked state
static uint8_t usb_started = 0;   // whether USB device stack is started

// Generic debounce utility (shared for future buttons)
typedef struct {
  uint8_t stable_level;     // last stable raw level (0/1)
  uint8_t debounced_state;  // last debounced logical state (0=not pressed,1=pressed)
  uint32_t last_toggle_ms;  // last time raw changed
  uint32_t debounce_ms;     // threshold
  uint8_t active_low;       // invert logic
} DebounceCtx;

static uint8_t debounce_update(DebounceCtx *ctx, uint8_t raw_level, uint32_t now_ms)
{
  uint8_t event = 0; // 1=pressed, 2=released
  if (raw_level != ctx->stable_level) {
    if ((now_ms - ctx->last_toggle_ms) >= ctx->debounce_ms) {
      // accept new stable level
      ctx->stable_level = raw_level;
      ctx->last_toggle_ms = now_ms;
      uint8_t logical = ctx->active_low ? (ctx->stable_level == 0) : (ctx->stable_level == 1);
      if (logical != ctx->debounced_state) {
        ctx->debounced_state = logical;
        event = logical ? 1 : 2;
      }
    }
  } else {
    // no change; refresh timer baseline
    ctx->last_toggle_ms = now_ms;
  }
  return event;
}

static DebounceCtx s_db_dose = { .stable_level = 1, .debounced_state = 0, .last_toggle_ms = 0, .debounce_ms = 30, .active_low = 1 };
static uint8_t usb_boot_check_active = 0; // non-blocking bootloader hold in progress
static uint32_t usb_boot_deadline_ms = 0;  // time when we decide (Jump or start USB)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void PlayBeep(uint32_t duration_ms);
void JumpToBootloader(void);
//void RTC_SetBuildTime(void);
//void Check_And_Log(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// removed extra helper prototypes; logic is inlined in main loop
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
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPTIM1_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_LPTIM2_Init();
  MX_ADC1_Init();
  MX_SAI1_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  // Remove eager USB init; start based on PE4 (VBUS)
  // MX_USB_Device_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */
  // Initialize peripherals for sensors & display
#if ENABLE_EPD
  EPD_GPIO_Init();
  EPD_HW_Init();
  epd_inited = 1;
#endif
  HAL_GPIO_WritePin(PW_TMP_GPIO_Port, PW_TMP_Pin, GPIO_PIN_SET); // power temperature sensor
  HAL_Delay(5);
  TMP102_Init(&hi2c1);
  init_ramdisk();

  // Charger config for CR2032: disable charging, set low-battery thresholds
  charger_set_enabled(false);
  charger_batt_watchdog_config(2700, 200, 100); // sleep=2.7V, hyst=0.2V, watchdog band=0.1V

  // Initial USB state according to PE4
  usb_connected = (HAL_GPIO_ReadPin(USB_I_GPIO_Port, USB_I_Pin) == GPIO_PIN_SET);
  if (usb_connected && !usb_started) {
    // Start USB immediately to avoid host enumeration timeout
    USB_Device_Start();
    usb_started = 1;
    // If ENTER is held, schedule bootloader jump after 5s
    if (HAL_GPIO_ReadPin(ENT_GPIO_Port, ENT_Pin) == GPIO_PIN_RESET) {
      usb_boot_check_active = 1;
      usb_boot_deadline_ms = HAL_GetTick() + 5000u; // 5s window
    }
  }
  logger_set_usb_active(usb_connected);
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    MX_APPE_Process();
    /* USER CODE BEGIN WHILE */
    // removed old dose_event_pending handler; DOSE is handled by debounce below
    if (usb_event_pending) {
      usb_event_pending = 0;
      uint8_t vbus_now = (HAL_GPIO_ReadPin(USB_I_GPIO_Port, USB_I_Pin) == GPIO_PIN_SET);
      if (vbus_now && !usb_started) {
        // Start USB immediately
        USB_Device_Start();
        usb_started = 1;
        if (HAL_GPIO_ReadPin(ENT_GPIO_Port, ENT_Pin) == GPIO_PIN_RESET) {
          usb_boot_check_active = 1;
          usb_boot_deadline_ms = HAL_GetTick() + 5000u;
        }
      } else if (!vbus_now && usb_started) {
        USB_Device_Stop();
        usb_started = 0;
        usb_boot_check_active = 0;
      }
      usb_connected = vbus_now;
      logger_set_usb_active(usb_connected);
    }

    // Handle deferred bootloader decision
    if (usb_boot_check_active) {
      // If still holding after deadline -> jump
      if ((int32_t)(HAL_GetTick() - usb_boot_deadline_ms) >= 0) {
        JumpToBootloader();
        usb_boot_check_active = 0; // in case jump returns
      } else {
        // If user released early, cancel boot jump (USB already started)
        if (HAL_GPIO_ReadPin(ENT_GPIO_Port, ENT_Pin) != GPIO_PIN_RESET) {
          usb_boot_check_active = 0;
        }
      }
    }

    // Debounce DOSE and act on PRESS event
    uint8_t dose_event = debounce_update(&s_db_dose, (HAL_GPIO_ReadPin(DOSE_GPIO_Port, DOSE_Pin) == GPIO_PIN_SET) ? 1 : 0, HAL_GetTick());
    if (dose_event == 1) { // pressed
      RTC_TimeTypeDef sTime; RTC_DateTypeDef sDate;
      HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
      HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
      uint16_t yearFull = 2000u + sDate.Year;
      uint8_t mon = sDate.Month;
      uint8_t day = sDate.Date;
      uint8_t hour = sTime.Hours;
      uint8_t min = sTime.Minutes;
      float temp_f = 0.0f;
      int8_t temp_i = LOG_NO_TEMP;
      if (TMP102_ReadTemperature(&hi2c1, &temp_f) == HAL_OK) {
        temp_i = (int8_t)(temp_f + (temp_f >= 0 ? 0.5f : -0.5f));
      }
      uint8_t dose_units = 1; // fixed per request
      append_log_rotating(yearFull, mon, day, hour, min, dose_units, temp_i);
      CHARGER_Fast fast = charger_get_fast();
#if ENABLE_EPD
      if (!epd_inited) { EPD_GPIO_Init(); EPD_HW_Init(); epd_inited = 1; }
      char dateStr[16];
      char timeStr[16];
      char tempStr[16];
      char battStr[20];
      char chgStr[12];
      snprintf(dateStr, sizeof(dateStr), "%02u.%02u.%04u", (unsigned)day, (unsigned)mon, (unsigned)yearFull);
      snprintf(timeStr, sizeof(timeStr), "%02u:%02u", (unsigned)hour, (unsigned)min);
      if (temp_i == (int8_t)LOG_NO_TEMP) snprintf(tempStr, sizeof(tempStr), "T: /"); else snprintf(tempStr, sizeof(tempStr), "T: %dC", temp_i);
      if (fast.batt_mv > 0) snprintf(battStr, sizeof(battStr), "Bat: %ldmV", (long)fast.batt_mv); else snprintf(battStr, sizeof(battStr), "Bat: /");
      switch (fast.status) {
        case CHARGER_STATUS_CHARGING: strcpy(chgStr, "CHG"); break;
        case CHARGER_STATUS_IDLE: strcpy(chgStr, "IDLE"); break;
        case CHARGER_STATUS_FAULT: strcpy(chgStr, "FLT"); break;
        default: strcpy(chgStr, "?"); break;
      }
      Paint_NewImage(BlackImage, EPD_WIDTH, EPD_HEIGHT, ROTATE_90, WHITE);
      Paint_SetMirroring(MIRROR_VERTICAL);
      Paint_Clear(WHITE);
      Paint_DrawString_EN(5, 5, dateStr, &Font16, WHITE, BLACK);
      Paint_DrawString_EN(5, 30, timeStr, &Font24, WHITE, BLACK);
      Paint_DrawString_EN(5, 60, tempStr, &Font16, WHITE, BLACK);
      Paint_DrawString_EN(5, 80, battStr, &Font16, WHITE, BLACK);
      Paint_DrawString_EN(5, 100, chgStr, &Font16, WHITE, BLACK);
      EPD_Display(BlackImage);
#endif
      PlayBeep(100);
    }
    __WFI(); // inline idle sleep
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_CRS */
  /* Configure CRS to auto-trim HSI48 using LSE for reliable USB FS clock */
  __HAL_RCC_CRS_CLK_ENABLE();
  RCC_CRSInitTypeDef crs = {0};
  crs.Prescaler = RCC_CRS_SYNC_DIV1;
  crs.Source = RCC_CRS_SYNC_SOURCE_LSE;
  crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  crs.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(32768, 48000000); // LSE=32768Hz
  crs.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  crs.HSI48CalibrationValue = 0x40; // mid calibration
  HAL_RCCEx_CRSConfig(&crs);
  /* USER CODE END USB_CRS */
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLLSAI1.PLLN = 24;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADCCLK;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */
/////////////////this function generates acoustic signal on mcu GPIO pin via LPTIM//////////////////////
void PlayBeep(uint32_t duration_ms) {
	uint32_t period = 999;  // 1 kHz (f=clk 4MHz/999)
	uint32_t pulse = 500;   // 50% duty cycle (D=pulse/period)
	// Spusti časovač na generovanie PWM
    HAL_LPTIM_PWM_Start(&hlptim1, period, pulse);

    // Čakaj, kým uplynie trvanie zvuku
    HAL_Delay(duration_ms);

    // Zastav časovač
    HAL_LPTIM_PWM_Stop(&hlptim1);
}

////////////////////this function releases all peripherials and jumps into bootloader/////////////////////////
void JumpToBootloader(void) {


    // Interupt disable
    __disable_irq();

    // Reset USB
    USB->CNTR = 0x0003;

    // Pheripherial reset
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOA_CLK_DISABLE();

    HAL_LPTIM_DeInit(&hlptim1);
    HAL_RCC_DeInit();

    // systick reset
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
///////////////////end of bootloader function////////////////////////

/////////////////set build time function//////////////////////
void RTC_SetBuildTime(void) {
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    // Parse __DATE__ "Apr 28 2025"
    char build_date[] = __DATE__;
    char build_time[] = __TIME__;

    // Mesiace
    const char *months[] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };

    int month = 0;
    for (int i = 0; i < 12; i++) {
        if (strncmp(build_date, months[i], 3) == 0) {
            month = i + 1;
            break;
        }
    }

    int day, year, hour, minute, second;
    sscanf(build_date + 4, "%d %d", &day, &year);
    sscanf(build_time, "%d:%d:%d", &hour, &minute, &second);

    sDate.Year = year % 100; // RTC chce len posledné 2 číslice
    sDate.Month = month;
    sDate.Date = day;
    sDate.WeekDay = 1; // Ignorujeme, HAL to nastaví sám

    sTime.Hours = hour;
    sTime.Minutes = minute;
    sTime.Seconds = second;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}
///////////////////end of buildtime function////////////////////////

//////////////////datalogger function////////////////////
/*
void Check_And_Log(void)
{
    static uint8_t last_button_state = 1;
    static uint32_t last_debounce_time = 0;
    const uint32_t debounce_delay = 50; // 50 ms debounce

    uint8_t current_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);

    if (current_state != last_button_state)
    {
        last_debounce_time = HAL_GetTick(); // reset debounce timer
    }

    if ((HAL_GetTick() - last_debounce_time) > debounce_delay)
    {
        if (current_state == GPIO_PIN_RESET) // tlačidlo stlačené (aktívny stav)
        {
            dose = 5;  // Tu si môžeš dynamicky nastaviť dávku ako potrebuješ

            int8_t temp_int = (int8_t)(temperature + 0.5f); // zaokrúhlenie floatu na najbližšie celé číslo

            Log_SaveRecord(dose, temp_int);
        }
    }

    last_button_state = current_state;
}
*/
/////////////////////////////end of datalogger function//////////////////////

// EXTI events: mark pending flags for main loop processing
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == DOSE_Pin) {
    dose_event_pending = 1;
  } else if (GPIO_Pin == USB_I_Pin) {
    usb_event_pending = 1;
  }
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
#ifdef USE_FULL_ASSERT
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
