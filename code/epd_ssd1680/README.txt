epd_ssd1680 - SSD1680 E-paper Display STM32 HAL Driver
------------------------------------------------------

Použitie:
1. Pripoj SPI1: PB3(SCLK), PB5(MOSI)
   a GPIO: PB4(DC), PB6(RES), PB7(BUSY), PA15(CS)
2. V main.c includni:
       #include "epd_ssd1680.h"
3. Inicializuj:
       epd_init();
       epd_clear(0xFF);         // Vyčistí displej na bielo
       epd_draw_string(0, 0, "Hello STM32");
       epd_update();            // Zobrazí obsah