/*
 * stme32f303re_gpio_driver.h
 *
 *  Created on: Jul 27, 2022
 *      Author: alie
 */

#ifndef INC_STM32F303RE_GPIO_DRIVER_H_
#define INC_STM32F303RE_GPIO_DRIVER_H_

#include <stm32f303re.h>
#include <stdint.h>
/* Handle structure for gpio */

// step 1 create GPIO config
typedef struct
{
	uint8_t GPIO_PinNumber;			// @GPIO_PIN_NO
	uint8_t GPIO_PinMode;			// @GPIO_PIN_MODES
	uint8_t GPIO_PinSPeed;			// @GPIO_PIN_SPEED
	uint8_t GPIO_PinOpType;			// @GPIO_PIN_OUTPUT_TYPE
	uint8_t GPIO_PuPdControl;		// @GPIO_PUPD_SETTING
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

// step 1 create GPIO Handle
typedef struct
{
	//Create Handle structure
	GPIO_RegDef_t *pGPIOx;			//holds the base address of gpio port to which the pin belongs

	// Create configuration Structure
	GPIO_PinConfig_t GPIO_PinConfig	;	// holds GPIO pin configuration setting
}GPIO_Handle_t;


// step 2 Enable Clock
/*
 * peripheral Clock setup
 * Parameter Base Address / Enable or Disable
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi);
/*
 * Init and de_init
 *
 */
// Step 3 Define Init and Deint function
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); // Base Address of GPIO
/*
 * Read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriortyConfig(uint8_t IRQNumber, uint32_t IRQPeriorty);
void GPIO_IRQHandling(uint8_t PinNumber);

// GPIO PIN NUMBER @GPIO_PIN_NO
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

//GPIO pin MODE @GPIO_PIN_MODES
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

//GPIO OUTPUT TYPE @GPIO_PIN_OUTPUT_TYPE
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

// GPIO PIN OUTPUT SPEED @GPIO_PIN_SPEED
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_VERYHIGH	3

//GPIO PULL UP AND PULL DOWN Configuration GPIO_PUPD_SETTING
#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2




#endif /* INC_STM32F303RE_GPIO_DRIVER_H_ */
