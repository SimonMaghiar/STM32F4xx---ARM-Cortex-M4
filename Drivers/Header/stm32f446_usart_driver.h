/*
 * stm32f446_usart_driver.h
 *
 *  Created on: Sep 5, 2019
 *      Author: Simon
 */

#ifndef INC_STM32F446_USART_DRIVER_H_
#define INC_STM32F446_USART_DRIVER_H_

#include "stm32f466xx.h"

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * Definitions for using interrupt
 */
#define USART_READY	0
#define USART_BUSY_IN_TX	1
#define USART_BUSY_IN_RX	2


typedef struct
{
	uint8_t USART_Mode;				/*@USART_Mode */
	uint32_t USART_Baud;			/*@USART_Baud */
	uint8_t USART_NrOfStopBits;		/*@USART_NoOfStopBits */
	uint8_t USART_WordLength;		/*@USART_WordLength */
	uint8_t USART_ParityControl;	/*@USART_ParityControl*/
	uint8_t USART_HWFlowControl;	/*@USART_HWFlowControl */
}USART_Config;

typedef struct
{
	USART_Struct *pUSARTx;
	USART_Config	USART_Config;
	uint8_t TxRxState;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t Len;
}USART_Handle;


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_Struct *pUSARTx, uint8_t EnorDi);
void USART_SetBaudRate(USART_Struct *pUSARTx, uint32_t BaudRate);
/*
 * Init and De-init
 */
void USART_Init(USART_Handle *pUSARTHandle);
void USART_DeInit(USART_Struct *pUSARTx);


/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle *pUSARTx,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle *pHandle);

void USART_TX_CLOSE(USART_Handle *pHandle);
void USART_RX_CLOSE(USART_Handle *pHandle);
/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_Struct *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_Struct *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_Struct *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle *pUSARTHandle,uint8_t AppEv);


/*
 * Status register
 */
#define PE		0
#define FE		1
#define NF		2
#define ORE		3
#define IDLE	4
#define RXNE	5
#define TC		6
#define TXE		7
#define LBD		8
#define CTS		9

/*
 * Control register 1
 */

#define SBK 	0
#define RWU		1
#define RE		2
#define TE		3
#define IDLEIE	4
#define RXNEIE	5
#define TCIE	6
#define TXEIE	7
#define PEIE	8
#define PS		9
#define PCE		10
#define WAKE	11
#define M		12
#define UE		13
#define OVER8	15

/*
 * Control register 2
 */

#define LBDL	5
#define LBDIE	6
#define LBCL	8
#define CPHA	9
#define CPOL	10
#define CLKEN	11
#define STOP	12
#define LINEN	14

/*
 * Control register 3
 */

#define EIE		0
#define IREN	1
#define IRLP	2
#define HDSEL	3
#define NACK	4
#define	SCEN	5
#define DMAR	6
#define DMAT	7
#define RTSE	8
#define CTSE	9
#define CTSIE	10
#define ONEBIT	11

#endif /* INC_STM32F446_USART_DRIVER_H_ */
