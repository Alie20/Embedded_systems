/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Oct 1, 2022
 *      Author: alie
 */


#include <stdio.h>
#include <string.h>
#include <stm32f303re.h>

I2C_Handle_t I2C1Handle;
void delay(void)
{
	for (uint32_t i = 0 ; i <= 1000000000; i++);
}



/*
 * PB6 	I2C1_SCL
 * PB7	I2C1_SDA
 */

#define SLAVEADDR			0x68
uint8_t tx_buf[32] = "STM32 SLAVE MODE TESTING";

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_config.NOSTRETCH = Clock_stretching_enabled;
	I2C1Handle.I2C_config.ANAlOG_FILTER = Analog_filter_disabled;
	I2C1Handle.I2C_config.DIGITAL_FILTER = Digital_filter_disabled;
	I2C1Handle.I2C_config.Timing = 0x00F0D8FF; // from Timing Excel configuration set
	I2C1Handle.I2C_config.I2C_DeviceAddress = SLAVEADDR;

	I2C_Init(&I2C1Handle);
}

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSPeed = GPIO_SPEED_MEDIUM;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

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
uint8_t rcv_buf[32];

int main ()
{


	uint8_t commandcode = 0x51;
	I2C1_GPIOInits();

	I2C1_Inits();

	//I2C IRQ Configurations
	I2C_IRQInterruptConfig(NVIC_IRQ_I2C1_EV, Enable);
	I2C_IRQInterruptConfig(NVIC_IRQ_I2C1_ER, Enable);

	Button_GPIOInits();

	while(commandcode == I2C_SlaveRecieveData(&I2C1Handle));
	I2C_SlaveSendData(&I2C1Handle,sizeof(tx_buf));

	commandcode = 0x52;
	while(commandcode == I2C_SlaveRecieveData(&I2C1Handle));
	I2C_SlaveSendData(&I2C1Handle,strlen(char*)(tx_buf));






}
void I2C1_ER_IRQHandler()
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C1_EV_EXTI23_IRQHandler()
{
	I2C_EV_IRQHandling(&I2C1Handle);
}
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t event)
{
	if (event == I2C_EV_TX_CMPLT)
	{
		printf("Tx is completed\n");
	}
	else if (event == I2C_EV_RX_CMPLT)
	{

		printf("Rx is completed\n");
		rxComplet = Set;

	}
	else if (event == I2C_ERROR_NACKF)
	{
		printf("Error: Nack Failure\n");
		I2C_CloseSendData(pI2CHandle);
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		while(1);
	}
}


