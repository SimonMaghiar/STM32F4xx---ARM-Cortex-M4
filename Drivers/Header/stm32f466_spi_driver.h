/*
 * stm32f466_spi_driver.h
 *
 *  Created on: Aug 27, 2019
 *      Author: Simon
 */

#ifndef INC_STM32F466_SPI_DRIVER_H_
#define INC_STM32F466_SPI_DRIVER_H_

#include "stm32f466xx.h"

/*
 * @SPI_BusConfig
 */

#define FULL_DUPLEX 	1
#define HALF_DUPLEX 	2
#define SIMPLEX_RX_ONLY 3

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	2

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

/*
 * @CPOL
 */

#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW  0

/*
 * @CPHA
 */

#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW  0

/*
 * @SPI_SSM
 */

/*
 * @SPI_SSI
 */
#define SPI_SSI_HIGH 1
#define SPI_SS1_LOW  0

#define SPI_SSM_EN	1
#define SPI_SSM_DI	0   //when it's disabled, it means it is hardware slave management and you have to drive the slave select to low externaly



typedef struct
{
	uint8_t SPI_DeviceMode;	/* select whether the device acts like master or slave */
	uint8_t SPI_BusConfig;  /* Full-duplex/ Half-duplex/ Simplex */
	uint8_t SPI_SclkSpeed;	/* Prescaler clock speed */
	uint8_t SPI_DFF;		/* 8/16 bit format */
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;		/* Software slave management / hardware slave management */
	uint8_t SPI_SSI;
}SPI_Config;

typedef struct
{
	SPI_Struct	*pSPIx;		/* This holds the base address of SPI(x:1,2,3) peripheral */
	SPI_Config SPIConfig;
	uint8_t *pTxBuffer;		/* !<To store the app. Tx buffer addres>*/
	uint8_t *pRxBuffer;		/* !<To store the app. Rx buffer addres>*/
	uint32_t TxLen;			/* !<To store the app. Tx length>*/
	uint32_t RxLen;			/* !<To store the app. Rx length>*/
	uint8_t TxState;		/* !<To store the app. Tx state>*/
	uint8_t RxState;		/* !<To store the app. Rx state>*/
}SPI_Handle;

/**************************************************************************************
 *									APIs supported by this driver
 *				For more information about the APIs check the function definition
 **************************************************************************************
 */

/*
 * CLOCK Init
 */

void SPI_PeriClockControl(SPI_Struct *pSPIx, uint8_t Enable);

/*
 * SPI Configuration Initialization and de-Initialization
 */

void SPI_Init(SPI_Handle *pSPIHandle);
void SPI_DeInit(SPI_Struct *pSPIx);

void SPI_SSOEConfig(SPI_Struct *pSPIx,uint8_t En);  //SSOE is a bit in SPI_CR2 register. You need to set this bit when the MCU is in master mode and it uses hardware management.
/*
 * Data send and receive
 */
//these are blocking APIs
void SPI_SendData(SPI_Struct *pSPIx, uint8_t *pTxBuffer, uint32_t size );
void SPI_ReceiveData(SPI_Struct *pSPIx, uint8_t *pRxBuffer, uint32_t size );

//non-blocking APIs based on interrupt
uint8_t SPI_SendDataIRQ(SPI_Handle *pSPIHandle, uint8_t *pTxBuffer, uint32_t size );
uint8_t SPI_ReceiveDataIRQ(SPI_Handle *pSPIHandle, uint8_t *pRxBuffer, uint32_t size );

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t Enable);
void SPI_IRQPriorityConfig(IRQn_Type IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle *pHandle);

void SPI_ApplicationEventCallback(SPI_Handle *pHandle, uint8_t AppEv);
/*
 * SPI possible states
 */

#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_CMPLT	3
#define SPI_EVENT_CRC_CMPLT	4

/*
 * SPI Control register 1 (SPI_CR1)
 */
#define CPHA	0
#define CPOL	1
#define MSTR	2
#define BR		3
#define SPE		6
#define LSBFIRST	7
#define SSI		8
#define SSM		9
#define RXONLY	10
#define DFF		11
#define CRCNEXT	12
#define CRCEN	13
#define BIDIOEN 14
#define BIDIMODE	15

/*
 * SPI Control register 2 (SPI_CR2)
 */
#define RXDMAEN	0
#define TXDMAEN	1
#define SSOE	2
#define FRF		4
#define ERRIE	5
#define RXNEIE	6
#define TXEIE	7

/*
 * SPI status register (SPI_SR)
 */

#define RXNE	0
#define TXE		1
#define CHSIDE	2
#define UDR		3
#define CRCERR	4
#define MODF	5
#define OVR		6
#define BSY		7
#define FRE		8

#endif /* INC_STM32F466_SPI_DRIVER_H_ */
