/*
 * bootload.c
 *
 *  Created on: Nov 8, 2025
 *      Author: orgo
 */

#include "bootload.h"
#include "stm32wbxx.h"
#include "stm32wbxx_hal.h"
#include "lptim.h"

#define BOOT_ADD 0x1FFF0000u

void JumpToBootloader(void)
{
  uint32_t jump_address;
  void (*boot_load)(void);

  /* Keep interrupts disabled for the whole handover. */
  __disable_irq();

  /* Reset USB peripheral before jump. */
  USB->CNTR = 0x0003u;

  /* Disable GPIO clocks used by application peripherals. */
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOA_CLK_DISABLE();

  HAL_LPTIM_DeInit(&hlptim1);
  HAL_RCC_DeInit();

  /* Stop SysTick. */
  SysTick->CTRL = 0u;
  SysTick->LOAD = 0u;
  SysTick->VAL = 0u;

  /* Clear NVIC enable/pending bits. */
  for (uint32_t i = 0; i < (sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0])); i++) {
    NVIC->ICER[i] = 0xFFFFFFFFu;
    NVIC->ICPR[i] = 0xFFFFFFFFu;
  }

  /* Remap and vector switch to system memory (ROM bootloader). */
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
  SCB->VTOR = BOOT_ADD;
  __DSB();
  __ISB();

  jump_address = *(__IO uint32_t *)(BOOT_ADD + 4u);
  __set_MSP(*(__IO uint32_t *)BOOT_ADD);
  boot_load = (void (*)(void))jump_address;
  boot_load();
}
