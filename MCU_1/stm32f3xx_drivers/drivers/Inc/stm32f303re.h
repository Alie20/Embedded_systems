/*
 * stm32f303re.h
 *
 *  Created on: Jul 25, 2022
 *      Author: Alie Eldeen
 */

#ifndef INC_STM32F303RE_H_
#define INC_STM32F303RE_H_
#include <stdint.h>

/*
 * Processor specific Details
 */
// Arm Cortex M4 ISERx Registers
#define NVIC_ISER0				((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1				((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2				((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3				((volatile uint32_t*)0xE000E10C)

// Arm Cortex M4 ICERx Registers
#define NVIC_ICER0				((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1				((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2				((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3				((volatile uint32_t*)0XE000E18C)

// Arm Cortex M4 Periorty Register

#define NVIC_PR_BASE_ADDR		((volatile uint32_t*)0xE000E404)

#define NO_PR_BITS_IMPLEMENTED 	4
// step 1 define different memories in MCU
#define FLASH_BASEADDR		0x08000000U //Flash base Address
#define SRAM1_BASEADDR		0x20000000U //SRAM1 Base Address
#define CCMSRAM_BASEADDR	0x10000000U	//CCMSRAM Base Address
#define ROM					0x1FFFD800U	//ROM Base Address
#define SRAM 				SRAM1BASEADDR

// step 2 define different buses in MCU
// Peripheral BUSES APB1-APB2-AHB1-AHB2-AHB3
#define  PERIPH_BASEADDR		0x40000000U
#define  APB1_BASEADDR			PERIPH_BASEADDR
#define  APB2_BASEADDR			0x40010000U
#define  AHB1_BASEADDR			0x40020000U
#define  AHB2_BASEADDR			0x48000000U
#define  AHB3_BASEADDR			0x50000000U
#define  AHB4_BASEADDR			0x60000000U

// step 3 define different GPIO peripheral in MCU
// Define AHB2 (GPIOx Peripheral)
#define GPIOA_BASEADDR			(AHB2_BASEADDR + 0x00000000U)
#define GPIOB_BASEADDR			(AHB2_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR			(AHB2_BASEADDR + 0x800U)
#define GPIOD_BASEADDR			(AHB2_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR			(AHB2_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR			(AHB2_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR			(AHB2_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR			(AHB2_BASEADDR + 0x1C00U)

// Define RCC
#define RCC_BASEADDR			(AHB1_BASEADDR + 0x1000)

// step 3
// Define  APB1 (I2Cx-SPIx-UARTx-USARTx)

// step 1 define I2C baseAddress
#define I2C1_BASEADDR 			(APB1_BASEADDR + 0x5400U)
#define I2C2_BASEADDR 			(APB1_BASEADDR + 0x5800U)
#define I2C3_BASEADDR 			(APB1_BASEADDR + 0x7800U)
#define SPI2_BASEADDR 			(APB1_BASEADDR + 0x3800U)
#define SPI3_BASEADDR 			(APB1_BASEADDR + 0x3C00U)


#define USART2_BASEADDR 		(APB1_BASEADDR + 0x4400U)
#define USART3_BASEADDR 		(APB1_BASEADDR + 0x4800U)
#define UART4_BASEADDR 			(APB1_BASEADDR + 0x4C00U)
#define UART5_BASEADDR			(APB1_BASEADDR + 0x5000U)

// step 3
// Define APB2 (EXTI- SPIx-USARTx-SYSCFG)
#define EXTI_BASEADDR			(APB2_BASEADDR + 0x0400U)
#define SYSCFG_BASEADDR			(APB2_BASEADDR + 0x0000U)
#define SPI1_BASEADDR			(APB2_BASEADDR + 0x3000U)
#define SPI4_BASEADDR			(APB2_BASEADDR + 0x3C00U)

#define USART1_BASEADDR			(APB2_BASEADDR + 0x3800U)

// GPIOx register map and reset values
// step 4
// define structures of GPIO
typedef struct
{
	volatile uint32_t MODER;			//GPIO port mode register				//Address offset:0x00
	volatile uint32_t OTYPER;			//GPIO port output type register		//Address offset: 0x04
	volatile uint32_t OSPEEDR;			//GPIO port output speed register		//Address offset: 0x08
	volatile uint32_t PUPDR;			//GPIO port pull-up/pull-down register	//Address offset: 0x0C
	volatile uint32_t IDR;				//GPIO port input data register			//Address offset: 0x10
	volatile uint32_t ODR;				//GPIO port output data register		//Address offset: 0x14
	volatile uint32_t BSRR;				//GPIO port bit set/reset register		//Address offset: 0x18
	volatile uint32_t LCKR;				//GPIO port configuration lock register	//Address offset: 0x1C
	volatile uint32_t AFR[2];			//GPIO alternate function [0] low register	//Address offset: 0x20
										//GPIO alternate function [1] high register	//Address offset: 0x24
	volatile uint32_t BRR;				//GPIO port bit reset register			//Address offset: 0x28

}GPIO_RegDef_t;

#define GPIOA ( (GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB ( (GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC ( (GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD ( (GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE ( (GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF ( (GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG ( (GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH ( (GPIO_RegDef_t*) GPIOH_BASEADDR)


// peripheral base Address of GPIOx type casted to xxx_register


//RCC register map structure
//step 5
// define structure RCC
typedef struct
{
	volatile uint32_t CR;			//Clock control register				//Address offset: 0x00
	volatile uint32_t CFGR;			//Clock configuration register			//Address offset: 0x04
	volatile uint32_t CIR;			//Clock interrupt register (RCC_CIR)	//Address offset: 0x08
	volatile uint32_t APB2RSTR;		//APB2 peripheral reset register		//Address offset: 0x0C
	volatile uint32_t APB1RSTR;		//APB1 peripheral reset register		//Address offset: 0x10
	volatile uint32_t AHBENR;		//AHB peripheral clock enable register	//Address offset: 0x14
	volatile uint32_t APB2ENR;		//APB2 peripheral clock enable register	//Address: 0x18
	volatile uint32_t APB1ENR;		//APB1 peripheral clock enable register //Address: 0x1C
	volatile uint32_t BDCR;			//RTC domain control register			//Address offset: 0x20
	volatile uint32_t CSR;			//Control/status register				//Address: 0x24
	volatile uint32_t AHBRSTR;		//AHB peripheral reset register			//Address: 0x28
	volatile uint32_t CFGR2;		//Clock configuration register 2		//Address: 0x2C
	volatile uint32_t CFGR3;		//Clock configuration register 3		//Address: 0x30
}RCC_RegDef_t;

// step 4 define RCC very important
#define RCC 	((RCC_RegDef_t*)RCC_BASEADDR)
/*
 * peripheral register define structure of EXTI
 *
 */

typedef struct
{
	volatile uint32_t IMR1;				//Interrupt mask register  			// Address:0x00
	volatile uint32_t EMR1;				//Event mask register				// Address:0x04
	volatile uint32_t RTSR1;			//Rising trigger selection register // Address:0x08
	volatile uint32_t FTSR1;			//Falling trigger selection register// Address:0x0C
	volatile uint32_t SWIER1;			//Software interrupt event register // Address:0x10
	volatile uint32_t PR1;				//Pending register					// Address:0x14
}EXTI_RegDef_t;

#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)

typedef struct
{
	volatile uint32_t CFGR1;			// Address 0x00
	volatile uint32_t RCR;				// Address 0x04
	volatile uint32_t EXTICR[4];		// Address 0x08- 0x14
	volatile uint32_t CFGR2;			// Address 0x18
	volatile uint32_t CFGR3;			// Address 0x50
	volatile uint32_t CFGR4;			// Address 0x48

}SYSCFG_RegDef_t;

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
// step 6
// Enable / disable / Reset Peripherals
// Macro Function to enable GPIOX
#define GPIOA_PCLK_EN()			(RCC-> AHBENR |= (1<<17))		//Enable GPIOA
#define GPIOB_PCLK_EN()			(RCC-> AHBENR |= (1<<18))		//Enable GPIOB
#define GPIOC_PCLK_EN()			(RCC-> AHBENR |= (1<<19))		//Enable GPIOC
#define GPIOD_PCLK_EN()			(RCC-> AHBENR |= (1<<20))		//Enable GPIOD
#define GPIOE_PCLK_EN()			(RCC-> AHBENR |= (1<<21))		//Enable GPIOE
#define GPIOF_PCLK_EN()			(RCC-> AHBENR |= (1<<22))		//Enable GPIOF


// step 6
// Enable / disable / Reset Peripherals
// Macro Function to enable I2C

#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1<<21))		//Enable I2C1
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1<<22))		//Enable I2C2
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1<<30))		//Enable I2C3

// step 6
// Enable / disable / Reset Peripherals
// Macro Function to enable SPI
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1<<12))		//Enable SPI1
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1<<14))		//Enable SPI2
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1<<15))		//Enable SPI3
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1<<15))		//Enalbe SPI4

// step 6
// Enable / disable / Reset Peripherals
// Macro Function to enable USART
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1<<14))		//Enable USART1
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1<<17))		//Enable USART2
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1<<18))		//Enable USART3
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1<<19))		//Enable USART4
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1<<20))		//Enable USART5

// Macro Function to enable SYSCFG

#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |=(1<<0))			//Enable SYSCFG

////////////////////////////////////////////////////////////////////////////////////////////////////////////

// step 6
// Enable / disable / Reset Peripherals
// Macro Function to disable GPIOX
#define GPIOA_PCLK_DI()			(RCC-> AHBENR &= ~(1<<17))		//Disable GPIOA
#define GPIOB_PCLK_DI()			(RCC-> AHBENR &= ~(1<<18))		//Disable GPIOB
#define GPIOC_PCLK_DI()			(RCC-> AHBENR &= ~(1<<19))		//Disable GPIOC
#define GPIOD_PCLK_DI()			(RCC-> AHBENR &= ~(1<<20))		//Disable GPIOD
#define GPIOE_PCLK_DI()			(RCC-> AHBENR &= ~(1<<21))		//Disable GPIOE
#define GPIOF_PCLK_DI()			(RCC-> AHBENR &= ~(1<<22))		//Disable GPIOF

// step 6
// Enable / disable / Reset Peripherals
// Macro Function to Reset GPIOX
#define GPIOA_REG_RESET()		do {(RCC-> AHBRSTR |= (1<<17)); (RCC-> AHBRSTR &= ~(1<<17));}while(0)
#define GPIOB_REG_RESET()		do {(RCC-> AHBRSTR |= (1<<18)); (RCC-> AHBRSTR &= ~(1<<17));}while(0)
#define GPIOC_REG_RESET()		do {(RCC-> AHBRSTR |= (1<<19)); (RCC-> AHBRSTR &= ~(1<<17));}while(0)
#define GPIOD_REG_RESET()		do {(RCC-> AHBRSTR |= (1<<20)); (RCC-> AHBRSTR &= ~(1<<17));}while(0)
#define GPIOE_REG_RESET()		do {(RCC-> AHBRSTR |= (1<<21)); (RCC-> AHBRSTR &= ~(1<<17));}while(0)
#define GPIOF_REG_RESET()		do {(RCC-> AHBRSTR |= (1<<22)); (RCC-> AHBRSTR &= ~(1<<17));}while(0)


// Reset SPI
#define SPI1_REG_RESET()		do {RCC->APB2ENR |= (1<<12); (RCC-> APB2ENR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()		do {RCC->APB1ENR |= (1<<14); (RCC-> APB1ENR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()		do {RCC->APB1ENR |= (1<<15); (RCC-> APB1ENR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET()		do {RCC->APB2ENR |= (1<<15); (RCC-> APB2ENR &= ~(1<<15));}while(0)

#define I2C1_REG_RESET()		do {RCC->APB1ENR |= (1<<21); (RCC-> APB2ENR &= ~(1<<21));}while(0)
#define I2C2_REG_RESET()		do {RCC->APB1ENR |= (1<<22); (RCC-> APB1ENR &= ~(1<<22));}while(0)
#define I2C3_REG_RESET()		do {RCC->APB1ENR |= (1<<30); (RCC-> APB1ENR &= ~(1<<30));}while(0)


// Reset USART
#define USART1_REG_RESET()		do {RCC->APB2ENR |= (1<<14); (RCC->APB2ENR &= ~(1<<14));}while(0)
#define USART2_REG_RESET()		do {RCC->APB1ENR |= (1<<17); (RCC->APB1ENR &= ~(1<<17));}while(0)
#define USART3_REG_RESET()		do {RCC->APB1ENR |= (1<<18); (RCC->APB1ENR &= ~(1<<18));}while(0)
#define UART4_REG_RESET()		do {RCC->APB1ENR |= (1<<19); (RCC->APB1ENR &= ~(1<<19));}while(0)
#define UART5_REG_RESET()		do {RCC->APB1ENR |= (1<<20); (RCC->APB1ENR &= ~(1<<20));}while(0)


// step 6
// Enable / disable / Reset Peripherals
// Macro Function to disable I2C

#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1<<21))		//Disable I2C1
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1<<22))		//Disable I2C2
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~(1<<30))		//Disable I2C3


// step 6
// Enable / disable / Reset Peripherals
// Macro Function to disable SPI
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1<<12))		//Disable SPI1
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1<<14))		//Disable SPI2
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1<<15))		//Disable SPI3
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1<<15))		//Disable SPI4



// step 6
// Enable / disable / Reset Peripherals
// Macro Function to disable USART
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<14))		//Disable USART1
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<17))		//Disable USART2
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<18))		//Disable USART3
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1<<19))		//Disable USART4
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1<<20))		//Disable USART5

// Macro Function to disable SYSCFG

#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1<<0))		//Disable SYSCFG

// Macro to GPIO_ BASEADDR_TO_CODE

#define GPIO_BASEADDR_TO_CODE(x)	     	((x == GPIOA)? 0 :(x == GPIOB)? 1 :(x == GPIOC)? 2 :(x == GPIOD)? 3 :(x == GPIOE)? 4 :(x == GPIOF)? 5 :0)


/*
 * Interrupt Request (IRQ) for stm32f303re
 */

#define IRQ_NO_EXTI0 					6
#define IRQ_NO_EXTI1					7
#define IRQ_NO_EXTI2					8
#define IRQ_NO_EXTI3					9
#define IRQ_NO_EXTI4					10
#define IRQ_NO_EXTI5_9					23
#define IRQ_NO_EXTI10_15				40


/*
 * Periorty number macro
 */

#define NVIC_IRQ_PERIO1					1
#define NVIC_IRQ_PERIO2					2
#define NVIC_IRQ_PERIO3					3
#define NVIC_IRQ_PERIO4					4
#define NVIC_IRQ_PERIO5					5
#define NVIC_IRQ_PERIO6					6
#define NVIC_IRQ_PERIO7					7
#define NVIC_IRQ_PERIO8					8
#define NVIC_IRQ_PERIO9					9
#define NVIC_IRQ_PERIO10				10
#define NVIC_IRQ_PERIO11				11
#define NVIC_IRQ_PERIO12				12
#define NVIC_IRQ_PERIO13				13
#define NVIC_IRQ_PERIO14				14
#define NVIC_IRQ_PERIO15				15
// General Macros

/*
 * IRQ for spi
 */

#define NVIC_IRQ_SPI1					35
#define NVIC_IRQ_SPI2					36
#define NVIC_IRQ_SPI3					51
#define NVIC_IRQ_SPI4					84

/*
 * IRQ for I2C
 */
#define NVIC_IRQ_I2C1_EV				31
#define NVIC_IRQ_I2C1_ER				32

#define NVIC_IRQ_I2C2_EV				33
#define NVIC_IRQ_I2C2_ER				34

#define NVIC_IRQ_I2C3_EV				72
#define NVIC_IRQ_I2C3_ER				73


#define Enable 							1
#define Disable 						0
#define Set 							Enable
#define Reset							Disable
#define GPIO_PIN_SET					Set
#define GPIO_PIN_RESET					Reset
#define FLAG_RESET						Reset
#define FLAG_SET						Set
#define Clock_stretching_enabled 		0
#define Clock_stretching_disabled 		1
#define Analog_filter_enabled			Enable
#define Analog_filter_disabled			Disable
#define Digital_filter_enabled			Enable
#define Digital_filter_disabled			Disable
#define I2C_Disable_SR					Disable
#define I2C_Enable_SR					Enable


// Step 1 define SPI Register in Struct
typedef struct
{
	volatile uint32_t SPI_CR1;      	//Control Register 1				offset 0x00
	volatile uint32_t SPI_CR2;			//Control Register 2				offset 0x04
	volatile uint32_t SPI_SR;			//Status Register       			offset 0x08
	volatile uint32_t SPI_DR;			//SPI data register     			offset 0x0C
	volatile uint32_t SPI_CRCPR;    	//Polynomial Register   			offset 0x10
	volatile uint32_t SPI_RXCRCR;		//Rx CRC register	    			offset 0x14
	volatile uint32_t SPI_TXCRCR;		//SPI Tx CRC register				offset 0x18
	volatile uint32_t SPI_I2SCFGR;		//SPIx_I2S configuration register	offset 0x1C
	volatile uint32_t SPI_I2SPR;		//SPIx_I 2 S pre-scaler register  	offset 0x20

}SPI_RegDef_t;

#define SPI1 ( (SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2 ( (SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3 ( (SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4 ( (SPI_RegDef_t*) SPI4_BASEADDR)

typedef struct
{
	volatile uint32_t I2C_CR1;      //Control Register 1				offset 0x00
	volatile uint32_t I2C_CR2;		//Control Register 2				offset 0x04
	volatile uint32_t I2C_OAR1;		//Own address Register1				offset 0x08
	volatile uint32_t I2C_OAR2;		//Own address Register2				offset 0x0C
	volatile uint32_t I2C_TIMINGR;	//Timing Register					offset 0x10
	volatile uint32_t I2C_TIMEOUTR;	//Timeout Register 					offset 0x14
	volatile uint32_t I2C_ISR;		//Interrupt and status Register		offset 0x18
	volatile uint32_t I2C_ICR;		//Interrupt clear register 			offset 0x1C
	volatile uint32_t I2C_PECR;		//PEC Register						offset 0x20
	volatile uint32_t I2C_RXDR;		//Receive Data register				offset 0x24
	volatile uint32_t I2C_TXDR;		//Transmit Data register			offset 0x28
}I2C_RegDef_t;


#define I2C1 ( (I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2 ( (I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3 ( (I2C_RegDef_t*) I2C3_BASEADDR)

typedef struct
{
	volatile uint32_t USART_CR1;    //Control Register 1					offset 0x00
	volatile uint32_t USART_CR2;	//Control Register 2					offset 0x04
	volatile uint32_t USART_CR3;	//Control Register 3					offset 0x08
	volatile uint32_t USART_BRR;	//Baud Rate Register					offset 0x0C
	volatile uint32_t USART_GTPR;
	volatile uint32_t USART_RTOR;
	volatile uint32_t USART_RQR;
	volatile uint32_t USART_ISR;	//Interrupt and status Register 		offset 0x1C
	volatile uint32_t USART_ICR;	//Interrupt Clear Register				offset 0x20
	volatile uint32_t USART_RDR;	//Receive Data register					offset 0x24
	volatile uint32_t USART_TDR;	//Transmit Data register				offset 0x28

}USART_RegDef_t;

#define USART1 		(USART_RegDef_t*)(USART1_BASEADDR)
#define USART2 		(USART_RegDef_t*)(USART2_BASEADDR)
#define USART3 		(USART_RegDef_t*)(USART3_BASEADDR)
#define UART4 		(USART_RegDef_t*)(UART4_BASEADDR)
#define UART5 		(USART_RegDef_t*)(UART5_BASEADDR)


/*
	//Guard time and pre-scaler Register	offset 0x10
	volatile uint32_t USART_RTOR;	//Receiver Timeout Register 			offset 0x14
	volatile uint32_t  * Bit position definition SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB_FIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_CRCL		11
#define SPI_CR1_CRC_NEXT	12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDI_OE		14
#define SPI_CR1_BIDI_MODE	15

/*
 * Bit Position defination SPI_CR2
 */

#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_NSSP				3
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7
#define SPI_CR2_DS					8
#define SPI_CR2_FRXT				12
#define SPI_CR2_LDMA_RX				13
#define SPI_CR2_LDMA_TX				14

/*
 * Bit position definition SPI_Status_register
 */

#define SPI_ISR_RXNE				0
#define SPI_ISR_TXE					1
#define SPI_ISR_CHSIDE				2
#define SPI_ISR_UDR					3
#define SPI_ISR_CRCERR				4
#define SPI_ISR_MODF				5
#define SPI_ISR_OVR					6
#define SPI_ISR_BSY					7
#define SPI_ISR_FRE					8
#define SPI_ISR_FRLVL				9
#define SPI_ISR_FTVL				11


/*
 * Bit position definition I2C_CR1
 */


#define I2C_CR1_PE				0
#define I2C_CR1_TXIE			1
#define I2C_CR1_RXIE			2
#define I2C_CR1_ADDRIE			3
#define I2C_CR1_NACKIE			4
#define I2C_CR1_STOPIE			5
#define I2C_CR1_TCIE			6
#define I2C_CR1_ERRIE			7
#define I2C_CR1_DNF				8
#define I2C_CR1_ANFOFF			12
#define I2C_CR1_TXDMAEN			14
#define I2C_CR1_RXDMAEN			15
#define I2C_CR1_SBC				16
#define I2C_CR1_NOSTRETCH		17
#define I2C_CR1_WUPEN			18
#define I2C_CR1_GCEN			19
#define I2C_CR1_SMBHEN			20


/*
 * Bit position definition I2C_CR2
 */

#define I2C_CR2_SADD			0
#define I2C_CR2_RD_WRN			10
#define I2C_CR2_ADD10			11
#define I2C_CR2_HEAD10R			12
#define I2C_CR2_START			13
#define I2C_CR2_STOP			14
#define I2C_CR2_NACK			15
#define I2C_CR2_NBYTES			16
#define I2C_CR2_RELOAD			24
#define I2C_CR2_AUTOEND			25
#define I2C_CR2_PECBYTE			26

/*
 * Bit position definition I2C_OSR1
 */
#define I2C_OAR1_OA1			0
#define I2C_OAR1_ADD7_1			1
#define I2C_OAR1_ADD9_8			8
#define I2C_OAR1_OA1MODE		10
#define I2C_OAR1_OA1EN			15

/*
 * Bit position definition I2C_OAR2
 */

#define I2C_OAR2_OA2			1
#define I2C_OAR2_OA2MSK			8
#define I2C_OAR2_OA2EN			15

/*
 * Bit position definition I2C_ISR
 */

#define I2C_ISR_TXE				0
#define I2C_ISR_TXIS			1
#define I2C_ISR_RXNE			2
#define I2C_ISR_ADDR			3
#define I2C_ISR_NACKF			4
#define I2C_ISR_STOPF			5
#define I2C_ISR_TC				6
#define I2C_ISR_TCR				7
#define I2C_ISR_BERR			8
#define I2C_ISR_ARLO			9
#define I2C_ISR_OVR				10
#define I2C_ISR_PECERR			11
#define I2C_ISR_TIMOUT			12
#define I2C_ISR_ALERT			13
#define I2C_ISR_BUSY			15
#define I2C_ISR_DIR				16
#define I2C_ISR_ADDCODE			17




/*
 * Bit position definition I2C_ICR
 */

#define I2C_ICR_ADDRCF				3
#define I2C_ICR_NACKCF				4
#define I2C_ICR_STOPCF				5
#define I2C_ICR_BERRCF				8
#define I2C_ICR_OVRCF				9
#define I2C_ICR_TIMOUTCF			10
#define I2C_ICR_ARLOCF				11
#define I2C_ICR_PECCF				12
#define I2C_ICR_ALERTCF  			13

/*
 * Bit position definition I2C_TIMINGR
 */

#define I2C_TIMINGR_SCLL			0
#define I2C_TIMINGR_SCLH			8
#define I2C_TIMINGR_SCLDEL			20
#define I2C_TIMINGR_SDADEL			16
#define I2C_TIMINGR_PRESC			28


/*
 * Bit position definition USART_CR1
 */

#define USART_CR1_UE				0			//PE interrupt enable
#define USART_CR1_UESM				1			//Interrupt Enable
#define USART_CR1_RE				2			//Transmission complete interrupt enable
#define USART_CR1_TE				3			//Transmitter Enable
#define USART_CR1_IDLEIE			4			//IDLE interrupt enable
#define USART_CR1_RXNEIE			5			//RXNE interrupt enable
#define USART_CR1_TCIE				6			//Transmission complete interrupt enable
#define USART_CR1_TXEIE				7			//Interrupt Enable
#define USART_CR1_PEIE				8			//PE interrupt enable
#define USART_CR1_PS				9			//parity selection
#define USART_CR1_PCE				10			//Parity control enable
#define USART_CR1_WAKE				11			//Receiver Wakeup method
#define USART_CR1_MO				12			//Word length
#define USART_CR1_MME				13			//Mute mode enable
#define USART_CR1_CMIE				14			//Character match interrupt enable
#define USART_CR1_OVER8				15			//Over8
#define USART_CR1_DEDT				16			//Driver Enable de-assertion time
#define USART_CR1_DEAT				21			//Driver Enable assertion time
#define USART_CR1_RTOIE				26			//Receiver timeout interrupt enable
#define USART_CR1_EOBIE				27			//End of block interrupt enable
#define USART_CR1_M1				28			//word length


/*
 * Bit position definition USART Control register 2
 */

#define USART_CR2_ADDM7				4			//7 bit address Detection/4 bit address Detection
#define USART_CR2_LBDL				5			//LIN break detection Length
#define USART_CR2_LBDIE				6			//LIN break detection interrupt enable
#define USART_CR2_LBCL				8			//Last bit clock pulse
#define USART_CR2_CPHA				9			//clock phase
#define USART_CR2_CPOL				10			//clock polarity
#define USART_CR2_CLKEN				11			//clock enable
#define USART_CR2_STOP				12			//Stop bits
#define USART_CR2_LINEN				14			//LIN Mode Enable
#define USART_CR2_SWAP				15			//Swap Tx/Rx pins
#define USART_CR2_RXINV				16			//RX Pin Active level Inversion
#define USART_CR2_TXINV				17			//Pin Active Level Inversion
#define USART_CR2_DATAINV			18			//Binary data inversion
#define USART_CR2_MSBFIRST			19			//Most signification bit first
#define USART_CR2_ABREN				20			//Auto baud rate enable
#define USART_CR2_ABRMOD			21			//Auto Baud rate mode
#define USART_CR2_RTOEN				23			//Receiver Timeout enable
#define USART_CR2_ADD3_0			24			//Address of the USART node
#define USART_CR2_ADD7_4			28			//Address of the USART node


/*
 * bit definition USART 3 control register
 */

#define USART_CR3_EIE					0		//Error Interrupt Enable
#define USART_CR3_IREN					1		//IrDA Mode Enable
#define USART_CR3_IRLP					2		//IrDA Low power
#define USART_CR3_HDSEL					3		//Half duplex Selection
#define USART_CR3_NACK					4		//smart card Nack Enable
#define USART_CR3_SCEN					5		//Smart card Mode enable
#define USART_CR3_DMAR					6		//DMA Enable Receiver
#define USART_CR3_DMAT					7		//DMA Enable transmitter
#define USART_CR3_RTSE					8		//RTS enable
#define USART_CR3_CTSE					9		//CTS enable
#define USART_CR3_CTSIE					10		//CTS interrupt enable
#define USART_CR3_ONEBIT				11		//One sample bit method enable
#define USART_CR3_OVRDIS				12		//Overrun Disable
#define USART_CR3_DDRE					13		//DMA Disable or Reception Error
#define USART_CR3_DEM					14		//Driver Enable mode
#define USART_CR3_DEP					15		//Driver enable polarity selection
#define USART_CR3_SCARCN				17		//smart card auto retry count
#define USART_CR3_WUS					20		//Wake up from stop mode interrupt flag selection
#define USART_CR3_WUFIE					22		//Wake up from stop mode interrupt enable

/*
 * bit definition USART Baud Rate register
 */

#define USART_BRR_BRR3_0				0
#define USART_BRR_BRR15_4				4



/*
 * bit definition USART Guard time and pre-scaler register
 */

#define USART_GTPR_PSC7_0				0
#define USART_GTPR_GT_7_0				8


/*
 * bit definition USART Receiver timeout register
 */

#define USART_RTOR_RTO15_0				0
#define USART_RTOR_RTO23_16				16
#define USART_RTOR_BLEN7_0				24



/*
 * bit definition USART Request Register
 */
#define USART_RQR_ABRRQ					0		//auto baud rate request
#define USART_RQR_SBKRQ					1		//Send break request
#define USART_RQR_MMRQ					2		//Mute mode request
#define USART_RQR_RXFRQ					3		//receive Data flush Request
#define USART_RQR_TXFRQ					4		//Transmit data flush request

/*
 * bit definition USART interrupt
 */

#define USART_ISR_PE					0
#define USART_ISR_FE					1
#define USART_ISR_NF					2
#define USART_ISR_ORE					3
#define USART_ISR_IDLE					4
#define USART_ISR_RXNE					5
#define USART_ISR_TC					6
#define USART_ISR_TXE					7
#define USART_ISR_LBDF					8
#define USART_ISR_CTSIF					9
#define USART_ISR_CTS					10
#define USART_ISR_RTOF					11
#define USART_ISR_EOBF					12
#define USART_ISR_ABRE					14
#define USART_ISR_ABRF					15
#define USART_ISR_BUSY					16
#define USART_ISR_CMF					17
#define USART_ISR_SBKF					18
#define USART_ISR_RWU					19
#define USART_ISR_WUF					20
#define USART_ISR_TEACK					21
#define USART_ISR_REACK					22
/*
 * bit definition USART Interrupt flag clear register
 */

#define USART_ICR_PECF					0		//Parity error clear flag
#define USART_ICR_FECF					1		//Framing Error clear flag
#define USART_ICR_NCF					2		//Noise detected clear flag
#define USART_ICR_ORECF					3		//Overrun error clear flag
#define USART_ICR_IDLECF				4		//Idle line detected clear flag
#define USART_ICR_TCCF					6		//Transmission complete clear flag
#define USART_ICR_LBDCF					8		//Lin Break detection clear flag
#define USART_ICR_CTSCF					9		//CTS Clear flag
#define USART_ICR_RTOCF					11		//Receiver timeout clear flag
#define USART_ICR_EOBCF					12		// End of block clear flag
#define USART_ICR_CMCF					17		// Character match clear flag
#define USART_ICR_WUCF					20		// Wake up from Stop mode clear flag


/*
 * bit definition USART Receive data register
 */

#define USART_RDR_RDR8_0				0		//Receive data value

/*
 * bit definition USART Transmit data register
 */
#define USART_TDR_TDR8_0				0		//Transmit data value

// Steps for SPI_drivers in stm32f303re.h
/*
 * Define SPI_Base_Address;
 * Enable Clock for SPI;
 * Disable Clock for SPI;
 * Create Struct for SPI Register;
 *
 */

#include "stm32f303re_gpio_driver.h"
#include "stm32f303re_SPI_driver.h"
#include <stm32f303re_I2C_driver.h>
#include "stm32f303re_USART_driver.h"
#include "stm32f303re_RCC_driver.h"

#endif /* INC_STM32F303RE_H_ */
