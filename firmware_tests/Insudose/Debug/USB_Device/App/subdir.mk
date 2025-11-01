################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_Device/App/usb_device.c \
../USB_Device/App/usbd_desc.c \
../USB_Device/App/usbd_storage_if.c 

OBJS += \
./USB_Device/App/usb_device.o \
./USB_Device/App/usbd_desc.o \
./USB_Device/App/usbd_storage_if.o 

C_DEPS += \
./USB_Device/App/usb_device.d \
./USB_Device/App/usbd_desc.d \
./USB_Device/App/usbd_storage_if.d 


# Each subdirectory must supply rules for building sources it contributes
USB_Device/App/%.o USB_Device/App/%.su USB_Device/App/%.cyclo: ../USB_Device/App/%.c USB_Device/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/Insudose/Drivers/Charger" -I../Core/Inc -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/Insudose/Drivers/EPD" -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/Insudose/Drivers/TMP102" -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/Insudose/Drivers/Logger" -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -I../FATFS/Target -I../FATFS/App -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USB_Device-2f-App

clean-USB_Device-2f-App:
	-$(RM) ./USB_Device/App/usb_device.cyclo ./USB_Device/App/usb_device.d ./USB_Device/App/usb_device.o ./USB_Device/App/usb_device.su ./USB_Device/App/usbd_desc.cyclo ./USB_Device/App/usbd_desc.d ./USB_Device/App/usbd_desc.o ./USB_Device/App/usbd_desc.su ./USB_Device/App/usbd_storage_if.cyclo ./USB_Device/App/usbd_storage_if.d ./USB_Device/App/usbd_storage_if.o ./USB_Device/App/usbd_storage_if.su

.PHONY: clean-USB_Device-2f-App

