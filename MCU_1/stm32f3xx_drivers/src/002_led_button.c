/*
 * 002_led_button.c
 *
 *  Created on: Aug 2, 2022
 *      Author: alie
 */


#include <stm32f303re.h>
#define Button_pressed	Disable
#define Button_released Enable
void delay (void)
{
	for(uint32_t i =0;i<100000; i ++);
}

int main(void)
{
	GPIO_Handle_t GpioLed,GpioButton;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSPeed = GPIO_SPEED_MEDIUM;
	GpioLed.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioLed.pGPIOx = GPIOA;

	GPIO_PeriClockControl(GPIOA, Enable);
	GPIO_Init(&GpioLed);

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	GpioButton.GPIO_PinConfig.GPIO_PinSPeed = GPIO_SPEED_MEDIUM;

	GPIO_PeriClockControl(GPIOA, Enable);
	GPIO_Init(&GpioButton);

	while(1)
	{
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_12) == 0)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}

	}

	return 0;
}
