################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/017rtc_lcd.c 

OBJS += \
./src/017rtc_lcd.o 

C_DEPS += \
./src/017rtc_lcd.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o src/%.su: ../src/%.c src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F3 -DSTM32F303RETx -DNUCLEO_F303RE -c -I../Inc -I/media/alie/af896648-84eb-46fb-8a34-aadf9fef82dd/Embedded/MCU1_Course/MCU_1/stm32f3xx_drivers/drivers/Inc -I"/media/alie/af896648-84eb-46fb-8a34-aadf9fef82dd/Embedded/MCU1_Course/MCU_1/stm32f3xx_drivers/bsp" -I"/media/alie/af896648-84eb-46fb-8a34-aadf9fef82dd/Embedded/MCU1_Course/MCU_1/stm32f3xx_drivers/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-src

clean-src:
	-$(RM) ./src/017rtc_lcd.d ./src/017rtc_lcd.o ./src/017rtc_lcd.su

.PHONY: clean-src

