/*
 * stm32f303re_RCC_driver.h
 *
 *  Created on: Oct 7, 2022
 *      Author: alie
 */

#ifndef INC_STM32F303RE_RCC_DRIVER_H_
#define INC_STM32F303RE_RCC_DRIVER_H_

#include <stm32f303re.h>
// implement CLK for APB1 BUS
uint32_t RCC_GetPCLK1Value(void);

// implement CLK for APB1 BUS
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F303RE_RCC_DRIVER_H_ */
