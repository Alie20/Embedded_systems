/*
 * 002_led_button.c
 *
 *  Created on: Aug 2, 2022
 *      Author: alie
 */


#include <stm32f303re.h>
#include <string.h>
#define Button_pressed	Disable
#define Button_released Enable
void delay (void)
{
	for(uint32_t i =0;i<100000; i ++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;
	memset(&GpioLed,0,sizeof(GpioLed));
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSPeed  = GPIO_SPEED_MEDIUM;
	GpioLed.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GPIOA, Enable);
	GPIO_Init(&GpioLed);

	GPIO_Handle_t  GpioButton;
	memset(&GpioLed,0,sizeof(GpioLed));
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioButton.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU;
	GpioButton.GPIO_PinConfig.GPIO_PinSPeed = GPIO_SPEED_MEDIUM;

	GPIO_PeriClockControl(GPIOA , Enable);
	GPIO_Init(&GpioButton);

/*

*/
//	IRQ Perority optional
	GPIO_IRQPriortyConfig(IRQ_NO_EXTI5_9,NVIC_IRQ_PERIO15);

//	IRQ Configuration
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI5_9, Enable);


 	while(1);
 	/*
	{
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_5) == 0)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_12);
		}

 	}
*/

//Go to startup_stm32f303retx.s and find IRQHANDLER of the EXTI corresponding to the line of the pin
}
void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_12);
}

