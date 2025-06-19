#include "epd_ssd1680.h"

int main(void) {
    epd_init();
    epd_clear(0xFF);
    epd_draw_string(0, 0, "Hello World!");
    epd_draw_string(0, 12, "SSD1680 Display");
    epd_update();
    while(1);
}
