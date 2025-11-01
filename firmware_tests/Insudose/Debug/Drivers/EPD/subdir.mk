################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/EPD/Display_EPD_W21.c \
../Drivers/EPD/Display_EPD_W21_spi.c \
../Drivers/EPD/GUI_Paint.c \
../Drivers/EPD/font12.c \
../Drivers/EPD/font12CN.c \
../Drivers/EPD/font16.c \
../Drivers/EPD/font20.c \
../Drivers/EPD/font24.c \
../Drivers/EPD/font24CN.c \
../Drivers/EPD/font8.c 

OBJS += \
./Drivers/EPD/Display_EPD_W21.o \
./Drivers/EPD/Display_EPD_W21_spi.o \
./Drivers/EPD/GUI_Paint.o \
./Drivers/EPD/font12.o \
./Drivers/EPD/font12CN.o \
./Drivers/EPD/font16.o \
./Drivers/EPD/font20.o \
./Drivers/EPD/font24.o \
./Drivers/EPD/font24CN.o \
./Drivers/EPD/font8.o 

C_DEPS += \
./Drivers/EPD/Display_EPD_W21.d \
./Drivers/EPD/Display_EPD_W21_spi.d \
./Drivers/EPD/GUI_Paint.d \
./Drivers/EPD/font12.d \
./Drivers/EPD/font12CN.d \
./Drivers/EPD/font16.d \
./Drivers/EPD/font20.d \
./Drivers/EPD/font24.d \
./Drivers/EPD/font24CN.d \
./Drivers/EPD/font8.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/EPD/%.o Drivers/EPD/%.su Drivers/EPD/%.cyclo: ../Drivers/EPD/%.c Drivers/EPD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/Insudose/Drivers/Charger" -I../Core/Inc -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/Insudose/Drivers/EPD" -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/Insudose/Drivers/TMP102" -I"C:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/Insudose/Drivers/Logger" -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -I../FATFS/Target -I../FATFS/App -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-EPD

clean-Drivers-2f-EPD:
	-$(RM) ./Drivers/EPD/Display_EPD_W21.cyclo ./Drivers/EPD/Display_EPD_W21.d ./Drivers/EPD/Display_EPD_W21.o ./Drivers/EPD/Display_EPD_W21.su ./Drivers/EPD/Display_EPD_W21_spi.cyclo ./Drivers/EPD/Display_EPD_W21_spi.d ./Drivers/EPD/Display_EPD_W21_spi.o ./Drivers/EPD/Display_EPD_W21_spi.su ./Drivers/EPD/GUI_Paint.cyclo ./Drivers/EPD/GUI_Paint.d ./Drivers/EPD/GUI_Paint.o ./Drivers/EPD/GUI_Paint.su ./Drivers/EPD/font12.cyclo ./Drivers/EPD/font12.d ./Drivers/EPD/font12.o ./Drivers/EPD/font12.su ./Drivers/EPD/font12CN.cyclo ./Drivers/EPD/font12CN.d ./Drivers/EPD/font12CN.o ./Drivers/EPD/font12CN.su ./Drivers/EPD/font16.cyclo ./Drivers/EPD/font16.d ./Drivers/EPD/font16.o ./Drivers/EPD/font16.su ./Drivers/EPD/font20.cyclo ./Drivers/EPD/font20.d ./Drivers/EPD/font20.o ./Drivers/EPD/font20.su ./Drivers/EPD/font24.cyclo ./Drivers/EPD/font24.d ./Drivers/EPD/font24.o ./Drivers/EPD/font24.su ./Drivers/EPD/font24CN.cyclo ./Drivers/EPD/font24CN.d ./Drivers/EPD/font24CN.o ./Drivers/EPD/font24CN.su ./Drivers/EPD/font8.cyclo ./Drivers/EPD/font8.d ./Drivers/EPD/font8.o ./Drivers/EPD/font8.su

.PHONY: clean-Drivers-2f-EPD

