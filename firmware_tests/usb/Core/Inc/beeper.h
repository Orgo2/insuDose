#ifndef BEEPER_H
#define BEEPER_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

void beeper_init(LPTIM_HandleTypeDef *timer);
void beeper_play(uint32_t duration_ms);
void beeper_tick(void);
bool beeper_is_active(void);

#endif /* BEEPER_H */
