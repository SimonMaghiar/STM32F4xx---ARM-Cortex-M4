/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Sep 5, 2019
 *      Author: Simon
 */

#include "stm32f446_usart_driver.h"
#include "stm32f466_rcc_driver.h"

void USART_PeriClockControl(USART_Struct *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_CLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_CLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_CLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_CLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_CLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_CLK_EN();
		}
	}
	else
	{
		//dissable USART/UART peripheral
	}
}

void USART_SetBaudRate(USART_Struct *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}

void USART_Init(USART_Handle *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx,ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << RE) | ( 1 << TE) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << M ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control
		tempreg |= ( 1 << PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NrOfStopBits << STOP;
	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		// enable CTS flow control
		tempreg |= ( 1 << CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		// enable RTS flow control
		tempreg |= (1<< RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ( ( 1 << CTSE) | (1<<RTSE) );
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/
	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);

}

void USART_SendData(USART_Handle *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	while(Len > 0)
	{
		//wait until TXE flag is set in the SR
		while(!(pUSARTHandle->pUSARTx->SR & (1<<TXE)));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				pTxBuffer++;
				pTxBuffer++;
				Len--;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
				Len--;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			pTxBuffer++;
			Len--;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! (pUSARTHandle->pUSARTx->SR & (1<<TC)));
}

void USART_ReceiveData(USART_Handle *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	while(Len>0)
	{
		//wait until RXNE flag is set in the SR
		while(!(pUSARTHandle->pUSARTx->SR & (1<<RXNE)));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
				Len--;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				 pRxBuffer++;
				 Len--;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = pUSARTHandle->pUSARTx->DR;
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = ((uint8_t) pUSARTHandle->pUSARTx->DR & 0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
			Len--;
		}
	}

}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber < 32)
		{
			NVIC->ISER[0] |= (1<<IRQNumber);
		}
		else if(IRQNumber >=32 &&IRQNumber < 64)
		{
			NVIC->ISER[1] |= (1<<(IRQNumber-32));
		}
		else if(IRQNumber >=64 && IRQNumber < 96)
		{
			NVIC->ISER[2] |= (1<<(IRQNumber-64));
		}
	}
	else
	{

	}
}

uint8_t USART_SendDataIT(USART_Handle *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pUSARTHandle->TxRxState;
	if(state == USART_READY)
	{
		pUSARTHandle->TxRxState = USART_BUSY_IN_TX;
		//enable TXEIE interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1<<TXEIE);

		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->Len = Len;
	}

	return state;
}

uint8_t USART_ReceiveDataIT(USART_Handle *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pUSARTHandle->TxRxState;
	if(state == USART_READY)
	{
		pUSARTHandle->TxRxState = USART_BUSY_IN_RX;
		//enable TXEIE interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1<<RXNEIE);

		pUSARTHandle->pTxBuffer = pRxBuffer;
		pUSARTHandle->Len = Len;
	}

		return state;
}


void USART_IRQHandling(USART_Handle *pHandle)
{
	uint8_t temp1,temp2;

	// TRANSMISSION****************************************************************************
	temp1 = pHandle->pUSARTx->CR1 & (1<<TXEIE); //check if interrupt is enables for transmitting
	temp2 = pHandle->pUSARTx->SR & (1<<TXE);	//check the TXE bit in the status register

	if(temp1 & temp2)
	{
		//if interrupt is enabled and TXE (transmit data register is empty)
		if(pHandle->Len > 0)
		{
			pHandle->pUSARTx->DR = *(pHandle->pTxBuffer++);
			pHandle->Len--;
		}
		else
		{
			//close communication
			USART_TX_CLOSE(pHandle);
		}
	}

	// RECEPTION*******************************************************************************
	temp1 = pHandle->pUSARTx->CR1 & (1<<RXNEIE); //check if interrupt is enables for receiving
	temp2 = pHandle->pUSARTx->SR & (1<<RXNE);	//check the RXNE bit in the status register

	if(temp1 & temp2)
	{
		if(pHandle->Len > 0)
		{
			*(pHandle->pRxBuffer++) = pHandle->pUSARTx->DR;
			pHandle->Len--;
		}
		else
		{
			USART_RX_CLOSE(pHandle);
		}
	}

}



void USART_TX_CLOSE(USART_Handle *pHandle)
{
	pHandle->pUSARTx->CR1 &= ~(1<<TXEIE);
	pHandle->TxRxState = USART_READY;
	pHandle->pTxBuffer = NULL;
	pHandle->Len = 0;
}

void USART_RX_CLOSE(USART_Handle *pHandle)
{
	pHandle->pUSARTx->CR1 &= ~(1<<RXNEIE);
	pHandle->TxRxState = USART_READY;
	pHandle->pRxBuffer = NULL;
	pHandle->Len = 0;
}


























