/*
 * stm32f303re_I2C_driver.h
 *
 *  Created on: Oct 15, 2022
 *      Author: alie
 */

#ifndef DRIVERS_INC_STM32F303RE_I2C_DRIVER_H_
#define DRIVERS_INC_STM32F303RE_I2C_DRIVER_H_

#include <stm32f303re.h>

typedef struct
{
	uint8_t  I2C_DeviceAddress;
	uint32_t I2C_SCLSpeed;
}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t	I2C_Config;
	uint8_t 		*rxbuffer;
	uint8_t 		*txbuffer;
	uint8_t			TxRxState;
	uint32_t 		TxLen;
	uint32_t 		RxLen;
	uint8_t 		DevAddress;
}I2C_Handle_t;

/*
 * Clk speed
 */
#define I2C_SCL_SPEED_SM10			10000
#define I2C_SCL_SPEED_SM100			100000
#define I2C_SCL_SPEED_FM			400000
#define I2C_SCL_SPEED_FMP			500000

// @I2C_STATE
#define I2C_STATE_READY					0
#define I2C_STATE_BUSY_RX				1
#define I2C_STATE_BUSY_TX				2

// Control Repeated start
#define I2C_RS_DISABLE					0
#define I2C_RS_ENABLE 					1

// @I2C_AckControl
// CR2 NACK (ONLY IN SLAVE MODE)
#define I2C_ACK_ENABLE					1
#define I2C_ACK_DISABLE					0


/*
 * I2c Related status flags
 */
#define I2C_FLAG_TXE		(1 << I2C_ISR_TXE)		// Transmit buffer
#define I2C_FLAG_RXNE  		(1 << I2C_ISR_RXNE)		// Receive buffer
#define I2C_FLAG_TXIS		(1 << I2C_ISR_TXIS)		// Transmit interrupt status
#define I2C_FLAG_ADDR		(1 << I2C_ISR_ADDR)		// Address Matched
#define I2C_FLAG_BUSY		(1 << I2C_ISR_BUSY)		// BUS Busy
#define I2C_FLAG_NACK		(1 << I2C_ISR_NACKF)	// Not Acknowledge received flag
#define I2C_FLAG_STOPF		(1 << I2C_ISR_STOPF)	// Stop flag detection
#define I2C_FLAG_TC			(1 << I2C_ISR_TC)		// Transfer complete
#define I2C_FLAG_TCR		(1 << I2C_ISR_TCR)		// Transfer Complete Reload
#define I2C_FLAG_BUS_ERROR	(1 << I2C_ISR_BERR)		// Bus error
#define I2C_FLAG_PECERR		(1 << I2C_ISR_PECERR)	// PEC error in Reception
#define I2C_FLAG_OVR		(1 << I2C_ISR_OVR)		// Overrun Underrun
#define I2C_FLAG_ARLO		(1 << I2C_ISR_ARLO)		// Arbitration Lost
#define I2C_FLAG_ALERT		(1 << I2C_ISR_ALERT)	// SMBUS Alert
#define I2C_FLAG_TIMEOUT	(1 << I2C_ISR_TIMOUT)	// Timeout or tlow detection flag
#define I2C_FLAG_DIR		(1 << I2C_ISR_DIR)		// Transfer Direction
#define I2C_ADDCODE			(1 << I2C_ISR_ADDCODE)	// Address Match code;


/*
 * I2C APP Event
 */



// Callback events
#define I2C_EVENT_TX_COMPLTETE			1
#define I2C_EVENT_RX_COMPLTETE			2
#define I2C_EVENT_NACK_RECIEVED			3
#define I2C_EVENT_ERROR_OVR				4
#define I2C_EVENT_ERROR_TIMEOUT			5
#define I2C_EVENT_ERROR_BERR			6
#define I2C_EVENT_SLAVE_TRANSMIT		7
#define I2C_EVENT_SLAVE_RECIEVE			8
#define I2C_EVENT_SLAVE_ADDR			9 // slave mode addr match


/*
 * Enable I2C Peripheral
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx , uint8_t EnorDi);


/*
 * Enable I2C Clock
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx , uint8_t EnorDi);


/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,uint32_t len, uint8_t slaveAddr, uint8_t rs);

void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t len, uint8_t slaveAddr, uint8_t rs);

// Data send and receive (interrupt)
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,uint32_t len, uint8_t slaveAddr);

uint8_t I2C_MasterRecieveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t len, uint8_t slaveAddr);

// IRQ Configuration and ISR handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


// Get flag
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

// Application callback
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,
		uint8_t AppEv);

void I2C_IRQSlaveConfig(I2C_Handle_t *pI2CHandle, uint8_t EnOrDi);
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t value);
uint8_t I2C_SlaveRecieveData(I2C_RegDef_t *pI2C);


#endif /* DRIVERS_INC_STM32F303RE_I2C_DRIVER_H_ */
