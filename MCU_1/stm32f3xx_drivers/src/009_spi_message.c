/*
 * 009_spi_message.c
 *
 *  Created on: Sep 14, 2022
 *      Author: alie
 */

#include<stdio.h>
#include<string.h>
#include "stm32f303re.h"
volatile uint8_t dataAvailable = 0;
volatile uint8_t rcvStop = 0;

void delay(void)
{
	for (uint32_t i = 0 ; i <= 500000/2; i++);
}
void SPI2_GPIOInts()
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSPeed = GPIO_SPEED_MEDIUM;
	//Clk
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(& SPIPins);
	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(& SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(& SPIPins);
	//Nss
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(& SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.SPIConfig.SPI_BusConfig  = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed  = SPI_SCLK_SPEED_DIV8;
	SPI2handle.SPIConfig.SPI_DFF        = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPHA       = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_CPOL       = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_SSM        = SPI_SSM_SW_DI; // Hardware

	SPI_Init(& SPI2handle);
}

void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t SpiPinInterrupt;
	SpiPinInterrupt.pGPIOx = GPIOA;
	SpiPinInterrupt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	SpiPinInterrupt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	SpiPinInterrupt.GPIO_PinConfig.GPIO_PinSPeed = GPIO_SPEED_MEDIUM;
	SpiPinInterrupt.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&SpiPinInterrupt);

	GPIO_IRQPriortyConfig(IRQ_NO_EXTI5_9, NVIC_IRQ_PERIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI5_9, Enable);

}
int main()
{
	Slave_GPIO_InterruptPinInit();
	SPI2_GPIOInts();
	SPI2_Inits();
	SPI_SSOE_Config(SPI2, Enable);
	SPI_IRQInterruptConfig(NVIC_IRQ_SPI1, Enable);

	while(1)
	{
		rcvStop = 0;

		while(! dataAvailable);

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI5_9, Disable);
		SPI_PeriClockControl(SPI2, Enable);

		while(!rcvStop)
		{
			while(SPI_SendDataIT(SPI2, pTxBuffer, 1));
		}
	}


}
