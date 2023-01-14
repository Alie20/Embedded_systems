/*
 * main.c
 *
 *  Created on: Aug 2, 2022
 *      Author: alie
 */
#include "stm32f303re.h"
void delay(void)
{
	for (uint32_t i = 0 ; i <= 1000000000; i++);
}
int main(void)
{
	// Setup Peripheral
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSPeed = GPIO_SPEED_MEDIUM;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	// Initialize clock
	GPIO_PeriClockControl(GPIOA, Enable);

	// Initialize GPIOx
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();

	}




	return 0;
}
