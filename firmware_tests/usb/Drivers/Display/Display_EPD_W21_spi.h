#ifndef _DISPLAY_EPD_W21_SPI_
#define _DISPLAY_EPD_W21_SPI_

#include "stm32wbxx_hal.h"

// --- Definície portov a pinov pre e-paper displej SSD1680 ---
// Tvoje zapojenie podľa popisu:
// CS  -> PA15
// SCK -> PB3 (SPI1_SCK) - pri bitbangingu aj normálne GPIO
// MOSI -> PB5 (SPI1_MOSI) - pri bitbangingu aj normálne GPIO
// DC -> PB4
// RST -> PB6
// BUSY -> PB7

#define EPD_CS_GPIO_Port    GPIOA
#define EPD_CS_Pin          GPIO_PIN_15

#define EPD_SCK_GPIO_Port   GPIOB
#define EPD_SCK_Pin         GPIO_PIN_3

#define EPD_MOSI_GPIO_Port  GPIOB
#define EPD_MOSI_Pin        GPIO_PIN_5

#define EPD_DC_GPIO_Port    GPIOB
#define EPD_DC_Pin          GPIO_PIN_4

#define EPD_RST_GPIO_Port   GPIOB
#define EPD_RST_Pin         GPIO_PIN_6

#define EPD_BUSY_GPIO_Port  GPIOB
#define EPD_BUSY_Pin        GPIO_PIN_7

// --- Makrá na ovládanie pinov pomocou HAL ---
#define EPD_W21_MOSI_0   HAL_GPIO_WritePin(EPD_MOSI_GPIO_Port, EPD_MOSI_Pin, GPIO_PIN_RESET)
#define EPD_W21_MOSI_1   HAL_GPIO_WritePin(EPD_MOSI_GPIO_Port, EPD_MOSI_Pin, GPIO_PIN_SET)

#define EPD_W21_CLK_0    HAL_GPIO_WritePin(EPD_SCK_GPIO_Port, EPD_SCK_Pin, GPIO_PIN_RESET)
#define EPD_W21_CLK_1    HAL_GPIO_WritePin(EPD_SCK_GPIO_Port, EPD_SCK_Pin, GPIO_PIN_SET)

#define EPD_W21_CS_0     HAL_GPIO_WritePin(EPD_CS_GPIO_Port, EPD_CS_Pin, GPIO_PIN_RESET)
#define EPD_W21_CS_1     HAL_GPIO_WritePin(EPD_CS_GPIO_Port, EPD_CS_Pin, GPIO_PIN_SET)

#define EPD_W21_DC_0     HAL_GPIO_WritePin(EPD_DC_GPIO_Port, EPD_DC_Pin, GPIO_PIN_RESET)
#define EPD_W21_DC_1     HAL_GPIO_WritePin(EPD_DC_GPIO_Port, EPD_DC_Pin, GPIO_PIN_SET)

#define EPD_W21_RST_0    HAL_GPIO_WritePin(EPD_RST_GPIO_Port, EPD_RST_Pin, GPIO_PIN_RESET)
#define EPD_W21_RST_1    HAL_GPIO_WritePin(EPD_RST_GPIO_Port, EPD_RST_Pin, GPIO_PIN_SET)

#define isEPD_W21_BUSY   (HAL_GPIO_ReadPin(EPD_BUSY_GPIO_Port, EPD_BUSY_Pin) == GPIO_PIN_SET)

// --- Funkcie ---
void SPI_Write(uint8_t value);
void EPD_W21_WriteDATA(uint8_t datas);
void EPD_W21_WriteCMD(uint8_t command);
void EPD_GPIO_Init(void); // teraz ponechaná a správne upravená

#endif // _DISPLAY_EPD_W21_SPI_
