#ifndef TMP102_H
#define TMP102_H

#include "main.h"  // alebo uprav podľa tvojho MCU série

// TMP102 I2C adresa (7-bitova posunuta pre HAL na 8-bit formát)
#define TMP102_I2C_ADDRESS     (0x48 << 1)  // Výrobné nastavenie: 0x48 (ak je ADD0=GND)

// Funkcie
HAL_StatusTypeDef TMP102_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef TMP102_ReadTemperature(I2C_HandleTypeDef *hi2c, float *temperature_celsius);

#endif // TMP102_H
