/*
 * stm32f446_i2c_driver.h
 *
 *  Created on: Sep 2, 2019
 *      Author: Simon
 */

#ifndef INC_STM32F446_I2C_DRIVER_H_
#define INC_STM32F446_I2C_DRIVER_H_

#include "stm32f466xx.h"

/*@I2C_SCLSpeed */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM	400000

/*@I2C_ACKControl */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*@I2C_FMDutyCycle */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

#define REPEATED_START_ENABLE	1  	//NOTE! We didn't use repeated start in our project which means that we don't generate stop bit after each function called (I2C_MasterSendData or I2C_MasterReceiveData)
#define REPEATED_START_DISABLE	0	//This is good to prevent other Masters to take over the bus, while another master is trying to communicate with a slave.
									//If you look at the logic analyzer, after each function, there's a stop bit generated, which may be bad in situations when there are many masters
									//in order to prevent this happening, add another parameter to the 2 functions, and only if REPEADED_START = 0 , generate a STOP BIT.

/*
 * I2C related status flags definitions
 */

#define I2C_TXE_FLAG		(1<<TxE)
#define I2C_RXNE_FLAG		(1<<RxNE)
#define I2C_SB_FLAG			(1<<SB)
#define I2C_ADDR_FLAG		(1<<ADDR)
#define I2C_BTF_FLAG		(1<<BTF)

#define I2C_BERR_FLAG		(1<<BERR)
#define I2C_ARLO_FLAG		(1<<ARLO)
#define I2C_AF_FLAG			(1<<AF)
#define I2C_OVR_FLAG		(1<<OVR)
#define I2C_TIMEOUT_FLAG 	(1<<TIMEOUT)
/*
 * Configuration structure for I2Cx peripheral
 */

typedef struct
{
	uint32_t I2C_SCLSpeed;		/*@I2C_SCLSpeed */
	uint8_t	 I2C_DeviceAddress;	/*It is mentioned by the user */
	uint8_t  I2C_ACKControl;	/*@I2C_ACKControl */
	uint16_t I2C_FMDutyCycle;	/*@I2C_FMDutyCycle */
}I2C_Config;

/*
 * Handle structure for I2Cx peripheral
 */

typedef struct
{
	I2C_Struct *pI2Cx;
	I2C_Config I2C_Config;
	//the followings are necessary for non-blocking apis (interrupt based I2C)
	uint8_t	*pTxBuffer;		/* !<To store the app. Tx buffer address> */
	uint8_t *pRxBuffer;		/* !<To store the app. Rx buffer address> */
	uint32_t TxLen;			/* !<To store Tx length> */
	uint32_t RxLen;			/* !<To store Rx length> */
	uint8_t  TxRxState;		/* !<To store Communication state ->refer to @Comm_State> */
	uint32_t DevAddr;		/* !<To store slave/device address> */
	uint32_t RxSize;		/* !<To store Rx size> */
	uint8_t  Sr;			/* !<To store repeated start value> */
}I2C_Handle;

/*
 * Enable I2C from CR2 register by setting PE bit to 1
 */
#define I2C1_ENABLE()	(I2C1->CR1 |= (1<<PE))
#define I2C2_ENABLE()	(I2C2->CR1 |= (1<<PE))
#define I2C3_ENABLE()	(I2C3->CR1 |= (1<<PE))

/**************************************************************************************
 *									APIs supported by this driver
 *				For more information about the APIs check the function definition
 **************************************************************************************
 */

/*
 * CLOCK Init
 */

void I2C_PeriClockControl(I2C_Struct *pI2Cx, uint8_t Enable);
uint8_t I2C_FlagStatus(I2C_Struct *pI2Cx,uint32_t FlagName);
/*
 * SPI Configuration Initialization and de-Initialization
 */

void I2C_Init(I2C_Handle *pI2CHandle);
void I2C_DeInit(I2C_Struct *pI2Cx);

/*
 * Data Send and Receive
 */

void I2C_MasterSendData(I2C_Handle *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr);

void I2C_SlaveSendData(I2C_Struct *pI2C,uint8_t data); //not implemented
uint8_t I2C_SlaveReceiveData(I2C_Struct *pI2C);	//not implemented

/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t Enable);
void I2C_IRQPriorityConfig(IRQn_Type IRQNumber, uint32_t IRQPriority);

void I2C_EV_IRQHandling(I2C_Handle *pI2CHandle);	//event handler
void I2C_ER_IRQHandling(I2C_Handle *pI2CHandle);	//error handler

/*
 * Non-blocking APIs
 */

uint8_t I2C_MasterSendDataIRQ(I2C_Handle *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIRQ(I2C_Handle *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);


void I2C_CloseSendData(I2C_Handle *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle *pI2CHandle);

#define I2C_READY		0
#define I2C_BUSY_IN_RX	1
#define I2C_BUSY_IN_TX	2



/*
 * Control register 1 (I2C_CR1) bit definition
 */

#define PE			0
#define SMBUS		1
#define	SMBTYPE		3
#define ENARP		4
#define ENPEC		5
#define ENGC		6
#define NOSTRETCH	7
#define START		8
#define STOP		9
#define ACK			10
#define POS			11
#define PEC			12
#define ALERT		13
#define SWRST		15

/*
 * Control register 2 (I2C_CR2) bit definition
 */
#define ITERR	8
#define ITEVT	9
#define ITBUF	10
#define DMA		11
#define LAST	12

/*
 * Status register 1(I2C_SR1) bit definition
 */
#define SB		0
#define ADDR	1
#define BTF		2
#define ADD10	3
#define STOPF	4
#define RxNE	6
#define TxE		7
#define BERR	8
#define ARLO	9
#define AF		10
#define OVR		11
#define PECERR	12
#define TIMEOUT	14
#define SMBALERT 15

/*
 * Status register 2(I2C_SR2) bit definition
 */

#define MSL			0
#define BUSY		1
#define TRA			2
#define GENCALL		4
#define SMBDEFAULT	5
#define SMBHOST		6
#define DUALF		7

#endif /* INC_STM32F446_I2C_DRIVER_H_ */
