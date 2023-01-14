/*
 * 015uart_tx.c
 *
 *  Created on: Oct 8, 2022
 *      Author: alie
 */

#include <stdio.h>
#include <string.h>
#include <stm32f303re.h>
extern void initialise_monitor_handles(void);

USART_Handle_t USART2_Handle;
void delay(void)
{
	for (uint32_t i = 0 ; i <= 10000; i++);
}

char msg[1024] = "UART Tx[testing ...\n\r";
/*
 * PA2  USART2_TX
 * PA3  USART2_RX
 */

/*
 * Note Tx connect with Rx
 * 		Rx connect with Tx
 */

#define SLAVEADDR			0x68
void USART2_Inits(void)
{
	USART2_Handle.pUSARTx = USART2;
	USART2_Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART2_Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART2_Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART2_Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART2_Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART2_Handle.USART_Config.USART_HwFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&USART2_Handle);
}


void USART2_GPIOInits(void)
{
	GPIO_Handle_t USARTPins;

	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USARTPins.GPIO_PinConfig.GPIO_PinSPeed = GPIO_SPEED_HIGH;

	//SCL
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&USARTPins);

	//SDA
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&USARTPins);

}
void Button_GPIOInits(void)
{
	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU;
	GpioButton.GPIO_PinConfig.GPIO_PinSPeed = GPIO_SPEED_MEDIUM;

	GPIO_PeriClockControl(GPIOA, Enable);
	GPIO_Init(&GpioButton);

}

int main ()
{

	USART2_GPIOInits();

	USART_PeriClockControl(USART2, Enable);

	USART2_Inits();

	Button_GPIOInits();

	initialise_monitor_handles();

	printf("The application is running \n");

	while(1)
	{
		if (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_5))
		{
			delay();
			USART_ReceiveData(&USART2_Handle, pRxBuffer, Len)
			USART_SendData(&USART2_Handle, (uint8_t*)msg, strlen(msg));
		}
	}
	return 0;
}
