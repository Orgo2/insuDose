################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/EPD/Display_EPD_W21.c \
../Core/EPD/Display_EPD_W21_spi.c \
../Core/EPD/GUI_Paint.c \
../Core/EPD/font12.c \
../Core/EPD/font12CN.c \
../Core/EPD/font16.c \
../Core/EPD/font20.c \
../Core/EPD/font24.c \
../Core/EPD/font24CN.c \
../Core/EPD/font8.c 

OBJS += \
./Core/EPD/Display_EPD_W21.o \
./Core/EPD/Display_EPD_W21_spi.o \
./Core/EPD/GUI_Paint.o \
./Core/EPD/font12.o \
./Core/EPD/font12CN.o \
./Core/EPD/font16.o \
./Core/EPD/font20.o \
./Core/EPD/font24.o \
./Core/EPD/font24CN.o \
./Core/EPD/font8.o 

C_DEPS += \
./Core/EPD/Display_EPD_W21.d \
./Core/EPD/Display_EPD_W21_spi.d \
./Core/EPD/GUI_Paint.d \
./Core/EPD/font12.d \
./Core/EPD/font12CN.d \
./Core/EPD/font16.d \
./Core/EPD/font20.d \
./Core/EPD/font24.d \
./Core/EPD/font24CN.d \
./Core/EPD/font8.d 


# Each subdirectory must supply rules for building sources it contributes
Core/EPD/%.o Core/EPD/%.su Core/EPD/%.cyclo: ../Core/EPD/%.c Core/EPD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Orgo/STM32CubeIDE/workspace_1.16.1/bootloader/Core/EPD" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-EPD

clean-Core-2f-EPD:
	-$(RM) ./Core/EPD/Display_EPD_W21.cyclo ./Core/EPD/Display_EPD_W21.d ./Core/EPD/Display_EPD_W21.o ./Core/EPD/Display_EPD_W21.su ./Core/EPD/Display_EPD_W21_spi.cyclo ./Core/EPD/Display_EPD_W21_spi.d ./Core/EPD/Display_EPD_W21_spi.o ./Core/EPD/Display_EPD_W21_spi.su ./Core/EPD/GUI_Paint.cyclo ./Core/EPD/GUI_Paint.d ./Core/EPD/GUI_Paint.o ./Core/EPD/GUI_Paint.su ./Core/EPD/font12.cyclo ./Core/EPD/font12.d ./Core/EPD/font12.o ./Core/EPD/font12.su ./Core/EPD/font12CN.cyclo ./Core/EPD/font12CN.d ./Core/EPD/font12CN.o ./Core/EPD/font12CN.su ./Core/EPD/font16.cyclo ./Core/EPD/font16.d ./Core/EPD/font16.o ./Core/EPD/font16.su ./Core/EPD/font20.cyclo ./Core/EPD/font20.d ./Core/EPD/font20.o ./Core/EPD/font20.su ./Core/EPD/font24.cyclo ./Core/EPD/font24.d ./Core/EPD/font24.o ./Core/EPD/font24.su ./Core/EPD/font24CN.cyclo ./Core/EPD/font24CN.d ./Core/EPD/font24CN.o ./Core/EPD/font24CN.su ./Core/EPD/font8.cyclo ./Core/EPD/font8.d ./Core/EPD/font8.o ./Core/EPD/font8.su

.PHONY: clean-Core-2f-EPD

