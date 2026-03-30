################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
C_SRCS += \
../Drivers/RTC/rtc_driver.c

OBJS += \
./Drivers/RTC/rtc_driver.o

C_DEPS += \
./Drivers/RTC/rtc_driver.d


# Each subdirectory must supply rules for building sources it contributes
Drivers/RTC/%.o Drivers/RTC/%.su Drivers/RTC/%.cyclo: ../Drivers/RTC/%.c Drivers/RTC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/RTC -I../Drivers/Charger -I../USB_Device/Logger -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-RTC

clean-Drivers-2f-RTC:
	-$(RM) ./Drivers/RTC/rtc_driver.cyclo ./Drivers/RTC/rtc_driver.d ./Drivers/RTC/rtc_driver.o ./Drivers/RTC/rtc_driver.su

.PHONY: clean-Drivers-2f-RTC
