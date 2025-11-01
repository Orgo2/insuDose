################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/TMP102/tmp102.c 

OBJS += \
./Core/TMP102/tmp102.o 

C_DEPS += \
./Core/TMP102/tmp102.d 


# Each subdirectory must supply rules for building sources it contributes
Core/TMP102/%.o Core/TMP102/%.su Core/TMP102/%.cyclo: ../Core/TMP102/%.c Core/TMP102/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Orgo/STM32CubeIDE/workspace_1.16.1/bootloader/Core/EPD" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-TMP102

clean-Core-2f-TMP102:
	-$(RM) ./Core/TMP102/tmp102.cyclo ./Core/TMP102/tmp102.d ./Core/TMP102/tmp102.o ./Core/TMP102/tmp102.su

.PHONY: clean-Core-2f-TMP102

