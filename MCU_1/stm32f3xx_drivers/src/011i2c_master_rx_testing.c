/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Oct 1, 2022
 *      Author: alie
 */


#include <stdio.h>
#include <string.h>
#include <stm32f303re.h>

extern void initialise_monitor_handles(void);

I2C_Handle_t I2C1Handle;
void delay(void)
{
	for (uint32_t i = 0 ; i <= 1000; i++);
}



/*
 * PB6 	I2C1_SCL
 * PB7	I2C1_SDA
 */

#define SLAVEADDR			0x68
void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = SLAVEADDR;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM100;

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
	uint8_t len;
	initialise_monitor_handles();

	printf("The application is running \n");
	I2C1_GPIOInits();

	I2C1_Inits();

	Button_GPIOInits();

	while(1)
	{
		if (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_5))
		{
			delay();
			// Data Write
			I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVEADDR, I2C_RS_ENABLE);

			//Receive Data length
			I2C_MasterRecieveData(&I2C1Handle, &len, 1, SLAVEADDR, I2C_RS_ENABLE);
			printf("Length of the string is %d \n",len);

			commandcode = 0x52;
			I2C_MasterSendData(&I2C1Handle, &commandcode, 1,SLAVEADDR, I2C_RS_ENABLE);

			//Receive the whole data
			//Receive Data length
			I2C_MasterRecieveData(&I2C1Handle, rcv_buf, len, SLAVEADDR, I2C_RS_DISABLE);

			rcv_buf[len+1] = '\0';
			printf("Data : %s",rcv_buf);
		}
	}
	return 0;
}
