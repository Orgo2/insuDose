#ifndef EPD_SSD1680_H
#define EPD_SSD1680_H

#include "stm32wbxx_hal.h"
#include <stdint.h>

void epd_init(void);
void epd_clear(uint8_t color);
void epd_draw_char(uint8_t x, uint8_t y, char c);
void epd_draw_string(uint8_t x, uint8_t y, const char *str);
void epd_update(void);

#endif