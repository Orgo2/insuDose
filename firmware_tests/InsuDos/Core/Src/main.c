#include "main.h"
#include "stm32wbxx_hal_tim.h"  // To includes TIM and LPTIM definitions

#define BUTTON_PIN GPIO_PIN_0
#define BUTTON_PORT GPIOA
#define PIEZO_PIN GPIO_PIN_2
#define PIEZO_PORT GPIOB
#define BOOTLOADER_TIMEOUT_MS 2000

TIM_HandleTypeDef htim;  // Use TIM handle for LPTIM
uint32_t buttonPressStartTime = 0;
uint8_t buttonHeld = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM_Init(void);
void JumpToBootloader(void);
void PlayBeep(uint32_t duration_ms);
void PeriphCommonClock_Config(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    //PeriphCommonClock_Config();
    MX_GPIO_Init();
    MX_TIM_Init();

    while (1) {
        if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_RESET) {
            if (!buttonHeld) {
                buttonHeld = 1;
                buttonPressStartTime = HAL_GetTick();
            } else if (HAL_GetTick() - buttonPressStartTime >= BOOTLOADER_TIMEOUT_MS) {
                PlayBeep(1000); // Beep for 1 second
                JumpToBootloader();
            }
        } else {
            buttonHeld = 0; // Reset the flag if the button is not pressed
        }
    }
}

void JumpToBootloader(void) {
    void (*bootloader_start)(void);
    bootloader_start = (void (*)(void)) (*(volatile uint32_t*) (0x1FFF0004));
    __set_MSP(*(volatile uint32_t*) 0x1FFF0000);
    bootloader_start();
}

void MX_TIM_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    // Enable clock for TIM1 (or LPTIM1)
    __HAL_RCC_TIM1_CLK_ENABLE();  // LPTIM1 is often mapped to TIM1 or separate LPTIM1 registers depending on your MCU

    // Initialize TIM (which includes LPTIM1 in STM32WB)
    htim.Instance = TIM1;  // TIM1 (or LPTIM1) handle
    htim.Init.Prescaler = 79;  // Prescaler for 1 MHz clock
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = 249;  // Period for 4kHz frequency (1 MHz / 250)
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim) != HAL_OK) {
        Error_Handler();
    }

    // Clock source configuration for LPTIM (APB clock or low-speed oscillator)
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    // PWM configuration for LPTIM (using PWM mode)
    if (HAL_TIM_PWM_Init(&htim) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // Configure PWM channel
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 125;  // 50% duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1);
}

void PlayBeep(uint32_t duration_ms) {
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1);
    HAL_Delay(duration_ms);
    HAL_TIM_PWM_Stop(&htim, TIM_CHANNEL_1);
}

void Error_Handler(void) {
    while (1) {
        // Implement an error handler, like flashing an LED or something to indicate failure
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Konfigurácia základného oscilátora HSI (High-Speed Internal)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;  // Nezapíname PLL pre tento príklad
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();  // Volanie funkcie Error_Handler v prípade chyby
    }

    // Konfigurácia hodín pre procesor a periférie
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // Hlavná zbernica (AHB)
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;   // Periférie 1
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   // Periférie 2
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();  // Volanie funkcie Error_Handler v prípade chyby
    }
}
