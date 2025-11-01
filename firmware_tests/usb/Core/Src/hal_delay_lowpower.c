/*
 * hal_delay_lowpower.c
 *
 *  Created on: Aug 15, 2025
 *      Author: Orgo
 */
#include "stm32wbxx_hal.h"
#include "core_cm4.h"
/////nahradi hal delay funkciou ktora uspi kazdy 1ms tick mcu jadro pri cakani////
void HAL_Delay(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; // len SLEEP
  while ((HAL_GetTick() - t0) < ms) { __WFI(); }
}

