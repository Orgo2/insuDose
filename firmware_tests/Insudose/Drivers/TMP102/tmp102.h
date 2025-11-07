#ifndef TMP102_H
#define TMP102_H

#include "main.h"  // alebo uprav podľa tvojho MCU série

// TMP102 I2C adresa (7-bitova posunuta pre HAL na 8-bit formát)
#define TMP102_I2C_ADDRESS     (0x49 << 1)  // Výrobné nastavenie: 0x48 (ak je ADD0=GND)

// TMP102 register pointers
#define TMP102_REG_TEMPERATURE 0x00
#define TMP102_REG_CONFIG      0x01
#define TMP102_REG_T_LOW       0x02
#define TMP102_REG_T_HIGH      0x03

// Funkcie
HAL_StatusTypeDef TMP102_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef TMP102_ReadTemperature(I2C_HandleTypeDef *hi2c, float *temperature_celsius);
HAL_StatusTypeDef TMP102_SetAlarm(I2C_HandleTypeDef *hi2c, float t_low_c, float t_high_c, uint8_t fault_count);
void TMP102_Deinit();
#endif // TMP102_H
