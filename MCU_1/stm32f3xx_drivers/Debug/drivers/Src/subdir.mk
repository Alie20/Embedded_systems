################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f303re_I2C_driver.c \
../drivers/Src/stm32f303re_RCC_driver.c \
../drivers/Src/stm32f303re_SPI_driver.c \
../drivers/Src/stm32f303re_USART_driver.c \
../drivers/Src/stm32f303re_gpio_driver.c 

OBJS += \
./drivers/Src/stm32f303re_I2C_driver.o \
./drivers/Src/stm32f303re_RCC_driver.o \
./drivers/Src/stm32f303re_SPI_driver.o \
./drivers/Src/stm32f303re_USART_driver.o \
./drivers/Src/stm32f303re_gpio_driver.o 

C_DEPS += \
./drivers/Src/stm32f303re_I2C_driver.d \
./drivers/Src/stm32f303re_RCC_driver.d \
./drivers/Src/stm32f303re_SPI_driver.d \
./drivers/Src/stm32f303re_USART_driver.d \
./drivers/Src/stm32f303re_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F3 -DSTM32F303RETx -DNUCLEO_F303RE -c -I../Inc -I/media/alie/af896648-84eb-46fb-8a34-aadf9fef82dd/Embedded/MCU1_Course/MCU_1/stm32f3xx_drivers/drivers/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f303re_I2C_driver.d ./drivers/Src/stm32f303re_I2C_driver.o ./drivers/Src/stm32f303re_I2C_driver.su ./drivers/Src/stm32f303re_RCC_driver.d ./drivers/Src/stm32f303re_RCC_driver.o ./drivers/Src/stm32f303re_RCC_driver.su ./drivers/Src/stm32f303re_SPI_driver.d ./drivers/Src/stm32f303re_SPI_driver.o ./drivers/Src/stm32f303re_SPI_driver.su ./drivers/Src/stm32f303re_USART_driver.d ./drivers/Src/stm32f303re_USART_driver.o ./drivers/Src/stm32f303re_USART_driver.su ./drivers/Src/stm32f303re_gpio_driver.d ./drivers/Src/stm32f303re_gpio_driver.o ./drivers/Src/stm32f303re_gpio_driver.su

.PHONY: clean-drivers-2f-Src

