/*
 * stm32f303re_RCC_driver.c
 *
 *  Created on: Oct 7, 2022
 *      Author: alie
 */


#include <stm32f303re_RCC_driver.h>
uint32_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint32_t APB1_PreScaler[4]= {2,4,8,16};
uint32_t APB2_PreScaler[4] = {2,4,8,16};


uint32_t RCC_GetPLLOutputClock()
{
	return 1;
}
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,Systemclk;

	uint8_t clksrc, temp,ahbp,apb1p;

	// Checking the clock of the system
	clksrc = ((RCC->CFGR>>2) & 0x3);

	if (clksrc == 0)
	{
		Systemclk = 8000000;
	}
	else if (clksrc == 1)
	{
		Systemclk = 4000000;
	}
	else if (clksrc == 2)
	{
		Systemclk = RCC_GetPLLOutputClock();
	}

	// Checking the prescaler for AHB BUS
	temp = ((RCC->CFGR >>4) & 0xF);
	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}
	// Checking the prescaler for APB1 Bus
	temp = ((RCC->CFGR >>8) & 0x7);
	if (temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp -4];
	}

	pclk1 = ((Systemclk / ahbp) / apb1p);

	return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2,Systemclk;

		uint8_t clksrc, temp,ahbp,apb2p;

		// Checking the clock of the system
		clksrc = ((RCC->CFGR>>2) & 0x3);

		if (clksrc == 0)
		{
			Systemclk = 8000000;
		}
		else if (clksrc == 1)
		{
			Systemclk = 4000000;
		}
		else if (clksrc == 2)
		{
			Systemclk = RCC_GetPLLOutputClock();
		}

		// Checking the prescaler for AHB BUS
		temp = ((RCC->CFGR >>4) & 0xF);
		if (temp < 8)
		{
			ahbp = 1;
		}
		else
		{
			ahbp = AHB_PreScaler[temp - 8];
		}
		// Checking the prescaler for APB1 Bus
		temp = ((RCC->CFGR >>11) & 0x7);
		if (temp < 4)
		{
			apb2p = 1;
		}
		else
		{
			apb2p = APB2_PreScaler[temp -4];
		}

		pclk2 = ((Systemclk / ahbp) / apb2p);

		return pclk2;
}

