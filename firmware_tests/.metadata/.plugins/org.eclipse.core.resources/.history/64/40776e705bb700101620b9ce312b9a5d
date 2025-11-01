#include "tmp102.h"

// Inicializácia TMP102 (voliteľné, môžeš nakonfigurovať režimy ak chceš)
HAL_StatusTypeDef TMP102_Init(I2C_HandleTypeDef *hi2c)
{
    // TMP102 je pripravený ihneď po napájaní.
    // Voliteľne by si mohol nastaviť konfiguráciu, ale default je OK.
    return HAL_OK;
}

// Čítanie teploty
HAL_StatusTypeDef TMP102_ReadTemperature(I2C_HandleTypeDef *hi2c, float *temperature_celsius)
{
    uint8_t buffer[2];
    uint8_t reg_pointer = 0x00; // Pointer na Temperature Register
    HAL_StatusTypeDef ret;

    // Nastav pointer na temperature register
    ret = HAL_I2C_Master_Transmit(hi2c, TMP102_I2C_ADDRESS, &reg_pointer, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ret;
    }

    // Prečítaj 2 bajty
    ret = HAL_I2C_Master_Receive(hi2c, TMP102_I2C_ADDRESS, buffer, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ret;
    }


    // Spracovanie údajov
    uint16_t raw_temp = (buffer[0] << 4) | (buffer[1] >> 4);

    // Ak je teplota záporná, rozšíriť znamienko
    if (raw_temp & 0x800) {
        raw_temp |= 0xF000;
    }

    // Prevod na °C
    *temperature_celsius = raw_temp * 0.0625f;

    return HAL_OK;
}
