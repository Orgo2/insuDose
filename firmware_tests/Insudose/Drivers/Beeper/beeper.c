/*
 * beeper.c
 *
 *  Created on: Nov 8, 2025
 *      Author: orgo
 */

#ifndef BEEPER_BEEPER_C_
#define BEEPER_BEEPER_C_

#include "beeper.h"
#include "lptim.h"

/////////////////this function generates acoustic signal on mcu GPIO pin via LPTIM//////////////////////
void PlayBeep_Internal(uint32_t duration_ms, uint32_t freq_hz, uint8_t pwm_percent) {
	// Obmedzenie PWM na maximum 50%
	if (pwm_percent > 50) {
		pwm_percent = 50;
	}
	
	// LPTIM clock je 4 MHz
	// period = (clock / freq) - 1
	uint32_t lptim_clock = 4000000;  // 4 MHz
	uint32_t period = (lptim_clock / freq_hz) - 1;
	
	// pulse = (period * pwm_percent) / 100
	uint32_t pulse = (period * pwm_percent) / 100;
	
	// Spusti časovač na generovanie PWM
    HAL_LPTIM_PWM_Start(&hlptim1, period, pulse);

    // Čakaj, kým uplynie trvanie zvuku
    HAL_Delay(duration_ms);

    // Zastav časovač
    HAL_LPTIM_PWM_Stop(&hlptim1);
}



#endif /* BEEPER_BEEPER_C_ */


