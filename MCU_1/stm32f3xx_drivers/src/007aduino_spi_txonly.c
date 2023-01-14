/*
 * 006spitx_testing.c
 *
 *  Created on: Sep 10, 2022
 *      Author: Alie Eldeen
 */
#include <string.h>
#include <stdio.h>
#include "stm32f303re.h"
extern void initialise_monitor_handles(void);

void delay()
{
	for (uint32_t i = 0 ; i<100000;i++);
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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(& SPIPins);

	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(& SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(& SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(& SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx= SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig  = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed  = SPI_SCLK_SPEED_DIV8;
	SPI2handle.SPIConfig.SPI_DFF        = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPHA       = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_CPOL       = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_SSM        = SPI_SSM_SW_DI; // Hardware

	SPI_Init(& SPI2handle);
}

void GPIO_Buttonint()
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
	/*
	 *  Exercise Test The SPI_SendData API to send the string "Hello World"
	 *  and use the below configurations
	 *  1. SPI 2 master
	 *  2. SCLK max Possible
	 *  3. DFF = 0 , DFF = 1
	 *  2 pins required
	 *  1. MOSI
	 *  2. SCLK
	 *
	 *  first we need to check the pins
	 *  from data sheet look at Alternate function mapped
	 *  PB12 : SPI2_NSS
	 *  PB13 : SPI2_SCLK
	 *  PB14 : MISO
	 *  PB15 : MOSI
	 *  Alternate function AF5
	 *   first step initialize the ports pin
	 *
	*/
	initialise_monitor_handles();
	GPIO_Buttonint();
	// data the will be sent
	char user_data[] = "I AM ALIE ELDEEN. I HOPE IT WORKS..";
	printf("The process is running\n");


	// Intialize the ports for SPI
	SPI2_GPIOInts();

	// Intialize SPI2
	SPI2_Inits();

	/*
	 * SSOE = 1 does NSS output enable
	 * The NSS pin is automatically manged by the hardawre
	 * i.e when SPE = 1 NSS will pulled to low
	 * and NSS pin will be high when SPE = 0
	 */
	// Enable SSOE
	SPI_SSOE_Config(SPI2 , Enable);
	while(1)
	{
		if (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_5))
		{
			delay();

			// we have to enable SPI peripheral After adding configuration not before because
			// when SPI peripheral enabled it didn't allow to edit in configuration.
			SPI_PeripheralControl(SPI2, Enable);
			uint8_t dataLen = strlen(user_data);
			printf("DataLen %i\n",dataLen);
			SPI_SendData(SPI2,&dataLen,1);
			// Send Data
			SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

			//check the spi is not busy
			while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

			// After sending data close the clock
			SPI_PeripheralControl(SPI2, Disable);

		}
	}


	return 0;
}
