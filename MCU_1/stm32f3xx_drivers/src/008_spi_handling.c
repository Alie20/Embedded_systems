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


// command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ 	0x51
#define COMMAND_LED_READ		0X52
#define	COMMAND_PRINT			0X53
#define COMMAND_ID_READ			0X54


#define LED_ON					1
#define LED_OFF					0

//Ardiuno analog pins
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

#define LED_PIN					9


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

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(& SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(& SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(& SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
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

uint8_t SPI_VerifyResponse (uint8_t ack)
{
	if (ack == 0xF5)
	{
		return 1;
	}
	else
	{
		return 0;
	}

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
	printf("The PROGRAM IS RUNNING\n");
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read = 0;
	// Initialize button
	GPIO_Buttonint();

	// Initialize the ports for SPI
	SPI2_GPIOInts();

	// Initialize SPI2
	SPI2_Inits();

	// Enable SSOE
	/*
	 * SSOE = 1 does NSS output enable
	 * The NSS pin is automatically manged by the hardawre
	 * i.e when SPE = 1 NSS will pulled to low
	 * and NSS pin will be high when SPE = 0
	 */
	SPI_SSOE_Config(SPI2 , Enable);
	uint8_t commandcode = COMMAND_LED_CTRL;
	uint8_t ack ;
	uint8_t args[2];
	while(1)
	{


		if(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_5))
		{
			delay();

			// we have to enable SPI peripheral After adding configuration not before because
			// when SPI peripheral enabled it didn't allow to edit in configuration.
			SPI_PeripheralControl(SPI2, Enable);


			SPI_SendData(SPI2,&commandcode,1);

			SPI_ReceiveData(SPI2, &dummy_read, 1);
			printf("%i\n",dummy_read);
			SPI_SendData(SPI2,&dummy_write,1);

			SPI_ReceiveData(SPI2, &ack, 1);

			if (SPI_VerifyResponse (ack))
			{
				args[0] = LED_PIN;
				args[1] = LED_ON;
				SPI_SendData(SPI2, args, 2);
			}
			//check the spi is not busy
			while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

			// After sending data close the clock
			SPI_PeripheralControl(SPI2, Disable);

		}

	}


	return 0;
}
