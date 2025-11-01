################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/shci.c 

OBJS += \
./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/shci.o 

C_DEPS += \
./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/shci.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/%.o Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/%.su Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/%.cyclo: ../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/%.c Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/Insudose/Drivers/Charger" -I../Core/Inc -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/Insudose/Drivers/EPD" -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/Insudose/Drivers/TMP102" -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/Insudose/Drivers/Logger" -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -I../FATFS/Target -I../FATFS/App -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_WPAN-2f-interface-2f-patterns-2f-ble_thread-2f-shci

clean-Middlewares-2f-ST-2f-STM32_WPAN-2f-interface-2f-patterns-2f-ble_thread-2f-shci:
	-$(RM) ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/shci.cyclo ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/shci.d ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/shci.o ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/shci.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_WPAN-2f-interface-2f-patterns-2f-ble_thread-2f-shci

