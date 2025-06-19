#include "epd_ssd1680.h"
#include "ascii_8x12.h"

// Hardcoded GPIO/SPI mappings
#define EPD_CS_GPIO_Port    GPIOA
#define EPD_CS_Pin          GPIO_PIN_15
#define EPD_DC_GPIO_Port    GPIOB
#define EPD_DC_Pin          GPIO_PIN_4
#define EPD_RES_GPIO_Port   GPIOB
#define EPD_RES_Pin         GPIO_PIN_6
#define EPD_BUSY_GPIO_Port  GPIOB
#define EPD_BUSY_Pin        GPIO_PIN_7
#define hspi_EPD            hspi1

extern SPI_HandleTypeDef hspi1;

#define WIDTH  184
#define HEIGHT 88
#define FB_SIZE (WIDTH * HEIGHT / 8)
static uint8_t framebuffer[FB_SIZE];

static void epd_write_command(uint8_t cmd) {
    HAL_GPIO_WritePin(EPD_DC_GPIO_Port, EPD_DC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EPD_CS_GPIO_Port, EPD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi_EPD, &cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(EPD_CS_GPIO_Port, EPD_CS_Pin, GPIO_PIN_SET);
}

static void epd_write_data(uint8_t data) {
    HAL_GPIO_WritePin(EPD_DC_GPIO_Port, EPD_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EPD_CS_GPIO_Port, EPD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi_EPD, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(EPD_CS_GPIO_Port, EPD_CS_Pin, GPIO_PIN_SET);
}

static void epd_wait_until_idle(void) {
    while (HAL_GPIO_ReadPin(EPD_BUSY_GPIO_Port, EPD_BUSY_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(10);
    }
}

void epd_init(void) {
    HAL_GPIO_WritePin(EPD_RES_GPIO_Port, EPD_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(EPD_RES_GPIO_Port, EPD_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    epd_wait_until_idle();
    epd_clear(0xFF);
    epd_update();
}

void epd_clear(uint8_t color) {
    for (int i = 0; i < FB_SIZE; i++) framebuffer[i] = color;
}

void epd_draw_char(uint8_t x, uint8_t y, char c) {
    if (c < 32 || c > 127) return;
    if (x > WIDTH - 8 || y > HEIGHT - 12) return;

    const uint8_t *char_data = font8x12[c - 32];
    for (int row = 0; row < 12; row++) {
        uint8_t line = char_data[row];
        for (int col = 0; col < 8; col++) {
            if (line & (1 << (7 - col))) {
                int index = (x + col) + (y + row) * WIDTH;
                framebuffer[index / 8] &= ~(1 << (7 - (index % 8)));
            }
        }
    }
}

void epd_draw_string(uint8_t x, uint8_t y, const char *str) {
    while (*str) {
        epd_draw_char(x, y, *str++);
        x += 8;
    }
}

void epd_update(void) {
    epd_write_command(0x10); // Write RAM
    for (int i = 0; i < FB_SIZE; i++) {
        epd_write_data(framebuffer[i]);
    }
    epd_write_command(0x12); // Display Refresh
    epd_wait_until_idle();
}