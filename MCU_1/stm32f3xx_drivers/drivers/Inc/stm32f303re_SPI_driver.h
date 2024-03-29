/*
 * stm32f303re.SPI_driver.h
 *
 *  Created on: Sep 10, 2022
 *      Author: alie
 */

#ifndef INC_STM32F303RE_SPI_DRIVER_H_
#define INC_STM32F303RE_SPI_DRIVER_H_

#include <stm32f303re.h>
#include <stdint.h>

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

// step 1 in SPI define SPI Handle structure
typedef struct
{
	SPI_RegDef_t	*pSPIx;
	SPI_Config_t	SPIConfig;
	uint8_t 		*pTxBuffer;		// to store the app Tx buffer address
	uint8_t			*pRxBuffer;		// to store the app. Rx buffer address
	uint32_t		TxLen;			// to store Tx len
	uint32_t		RxLen;			// to store Rx len
	uint8_t			TxState;		// To store Tx state
	uint8_t			RxState;		// to store Rx state
}SPI_Handle_t;

/*
 * SPI Device Mode
 */

#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * SPI BUS CONFIG
 */

#define SPI_BUS_CONFIG_FD			1
#define SPI_BUS_CONFIG_HD			2
#define SPI_BUS_SIMPLEX_RXONLY		3

/*
 * SPI Clock speed
 */

#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * SPI_DFF
 */

#define SPI_DFF_8BITS		0
#define	SPI_DFF_16BITS		1

/*
 * CPOL
 */

#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 * CPHA
 */

#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

/*
 * SPI_SSM
 */

#define SPI_SSM_SW_EN	1
#define SPI_SSM_SW_DI	0

/*
 * SPI related status flags definations
 */

#define SPI_TXE_FLAG	(1 << SPI_ISR_TXE)		// transmit buffer
#define SPI_RXNE_FLAG   (1 << SPI_ISR_RXNE)		// receive buffer
#define SPI_BUSY_FLAG	(1 << SPI_ISR_BSY)		// Busy Flag


/*
 * possible SPI application states
 */
#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

/*
 * possible SPI Application events
 */
#define SPI_EVENT_TX_COMPLT			1
#define SPI_EVENT_RX_COMPLT			2
#define SPI_EVENT_OVR_ERR			3
#define SPI_EVENT_CRC_ERR			4
/*
 * Enable peripheral for SPI
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi);

/*
 * Peripheral Clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data send and receive
 * Blocking and non blocking based
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriortyConfig(uint8_t IRQNumber, uint32_t IRQPeriorty);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag_name);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * Applications Callback
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t event);


/*
 * Enable and disable SSI
 *  */
void SPI_SSI_Config(SPI_RegDef_t *pSPIx , uint8_t EnorDi);
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx , uint8_t EnorDi);







/*
 * steps for SPI configuration in stm32f303re_SPI_driver.h
 * create configuration structure for SPI
 * create Handle structure for SPI
 * Create a prototype function to the following
 * init and deint
 * data send and receive
 * IRQ Configuration and handling
 *
 */



#endif /* INC_STM32F303RE_SPI_DRIVER_H_ */
