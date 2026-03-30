################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Display/Display_EPD_W21.c \
../Drivers/Display/Display_EPD_W21_spi.c \
../Drivers/Display/GUI_Paint.c \
../Drivers/Display/font12.c \
../Drivers/Display/font20.c \
../Drivers/Display/font24.c 

OBJS += \
./Drivers/Display/Display_EPD_W21.o \
./Drivers/Display/Display_EPD_W21_spi.o \
./Drivers/Display/GUI_Paint.o \
./Drivers/Display/font12.o \
./Drivers/Display/font20.o \
./Drivers/Display/font24.o 

C_DEPS += \
./Drivers/Display/Display_EPD_W21.d \
./Drivers/Display/Display_EPD_W21_spi.d \
./Drivers/Display/GUI_Paint.d \
./Drivers/Display/font12.d \
./Drivers/Display/font20.d \
./Drivers/Display/font24.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Display/%.o Drivers/Display/%.su Drivers/Display/%.cyclo: ../Drivers/Display/%.c Drivers/Display/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/Display -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Display

clean-Drivers-2f-Display:
	-$(RM) ./Drivers/Display/Display_EPD_W21.cyclo ./Drivers/Display/Display_EPD_W21.d ./Drivers/Display/Display_EPD_W21.o ./Drivers/Display/Display_EPD_W21.su ./Drivers/Display/Display_EPD_W21_spi.cyclo ./Drivers/Display/Display_EPD_W21_spi.d ./Drivers/Display/Display_EPD_W21_spi.o ./Drivers/Display/Display_EPD_W21_spi.su ./Drivers/Display/GUI_Paint.cyclo ./Drivers/Display/GUI_Paint.d ./Drivers/Display/GUI_Paint.o ./Drivers/Display/GUI_Paint.su ./Drivers/Display/font12.cyclo ./Drivers/Display/font12.d ./Drivers/Display/font12.o ./Drivers/Display/font12.su ./Drivers/Display/font20.cyclo ./Drivers/Display/font20.d ./Drivers/Display/font20.o ./Drivers/Display/font20.su ./Drivers/Display/font24.cyclo ./Drivers/Display/font24.d ./Drivers/Display/font24.o ./Drivers/Display/font24.su

.PHONY: clean-Drivers-2f-Display
