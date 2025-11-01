################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_Device/Logger/logger.c 

OBJS += \
./USB_Device/Logger/logger.o 

C_DEPS += \
./USB_Device/Logger/logger.d 


# Each subdirectory must supply rules for building sources it contributes
USB_Device/Logger/%.o USB_Device/Logger/%.su USB_Device/Logger/%.cyclo: ../USB_Device/Logger/%.c USB_Device/Logger/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/Orgo/STM32CubeIDE/workspace_1.16.1/usb/USB_Device/Logger" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USB_Device-2f-Logger

clean-USB_Device-2f-Logger:
	-$(RM) ./USB_Device/Logger/logger.cyclo ./USB_Device/Logger/logger.d ./USB_Device/Logger/logger.o ./USB_Device/Logger/logger.su

.PHONY: clean-USB_Device-2f-Logger

