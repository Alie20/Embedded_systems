/*
 * 006spitx_testing.c
 *
 *  Created on: Sep 10, 2022
 *      Author: alie
 */
#include <string.h>
#include <stdio.h>
#include "stm32f303re.h"

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

	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(& SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig  = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed  = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF        = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPHA       = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_CPOL       = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_SSM        = SPI_SSM_SW_EN;

	SPI_Init(& SPI2handle);
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
	// data the will be sent
	char user_data[] = "Hello World";
	// Intialize the ports for SPI
	SPI2_GPIOInts();

	// Intialize SPI2
	SPI2_Inits();

	// Set SSI 1 to avoid MODF error
	SPI_SSI_Config(SPI2,Enable);

	// we have to enable SPI perihperal After adding configuration not before because
	// when SPI peripheral enabled it didn't allow to edit in configuration.
	SPI_PeripheralControl(SPI2, Enable);

	// Send Data
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

	// After sending data close the clock
	SPI_PeripheralClockControl(SPI2, Disable);

	while(1);



	return 0;
}
