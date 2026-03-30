################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/TMP102/tmp102.c 

OBJS += \
./Drivers/TMP102/tmp102.o 

C_DEPS += \
./Drivers/TMP102/tmp102.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/TMP102/%.o Drivers/TMP102/%.su Drivers/TMP102/%.cyclo: ../Drivers/TMP102/%.c Drivers/TMP102/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/usb/Drivers/Charger" -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/usb/USB_Device/Logger" -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-TMP102

clean-Drivers-2f-TMP102:
	-$(RM) ./Drivers/TMP102/tmp102.cyclo ./Drivers/TMP102/tmp102.d ./Drivers/TMP102/tmp102.o ./Drivers/TMP102/tmp102.su

.PHONY: clean-Drivers-2f-TMP102

