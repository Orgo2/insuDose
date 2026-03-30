/*
 * beeper.h
 *
 *  Created on: Nov 8, 2025
 *      Author: orgo
 */

#ifndef BEEPER_BEEPER_H_
#define BEEPER_BEEPER_H_

#include <stdint.h>

// Default values
#define BEEPER_DEFAULT_FREQ_HZ 1000
#define BEEPER_DEFAULT_PWM_PERCENT 50

// Internal function
void PlayBeep_Internal(uint32_t duration_ms, uint32_t freq_hz, uint8_t pwm_percent);

// Macro for optional parameters- macro is defined to select correct function based on number of arguments

#define GET_MACRO(_1,_2,_3,NAME,...) NAME
#define PlayBeep(...) GET_MACRO(__VA_ARGS__, PlayBeep_3, PlayBeep_2, PlayBeep_1)(__VA_ARGS__)
#define PlayBeep_1(duration) PlayBeep_Internal(duration, BEEPER_DEFAULT_FREQ_HZ, BEEPER_DEFAULT_PWM_PERCENT)
#define PlayBeep_2(duration, freq) PlayBeep_Internal(duration, freq, BEEPER_DEFAULT_PWM_PERCENT)
#define PlayBeep_3(duration, freq, pwm) PlayBeep_Internal(duration, freq, pwm)


#endif /* BEEPER_BEEPER_H_ */
