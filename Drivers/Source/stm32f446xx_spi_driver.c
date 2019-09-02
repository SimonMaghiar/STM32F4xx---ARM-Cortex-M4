/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Aug 27, 2019
 *      Author: Simon
 */
#include "stm32f466_spi_driver.h"
#include "stm32f466xx.h"

//these are helper function and if you try to call them from main, it will throw an error
static void spi_txe_interrupt_handle(SPI_Handle *pHandle);  //static in this case makes the function private, only usable in this source file
static void spi_rxne_interrupt_handle(SPI_Handle *pHandle);
static void spi_ovr_interrupt_handle(SPI_Handle *pHandle);

void SPI_PeriClockControl(SPI_Struct *pSPIx, uint8_t Enable)
{
	if(Enable == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_Clock_En();
		}
		else if( pSPIx == SPI2)
		{
			SPI2_Clock_En();
		}
		else if( pSPIx == SPI3)
		{
			SPI3_Clock_En();
		}
		else if( pSPIx == SPI4)
		{
			SPI4_Clock_En();
		}
	}
	else
	{
		// dissable clock for the respective peripheral
	}
}

void SPI_SSOEConfig(SPI_Struct *pSPIx,uint8_t En)
{
	if(En == ENABLE)
	{
		pSPIx->CR2 |= (1<<2);
	}
	else
	{
		pSPIx->CR2 &= ~(1<< 2);
	}
}

void SPI_Init(SPI_Handle  *pSPIHandle)
{


	//0. Enable clock for the peripheral !!! DO NOT FORGET !! Otherwise you write into the register and the bits won't change!
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	/*
	 * NSS output enable (SSM=0,SSOE = 1): this configuration is only used when the
	   MCU is set as master. The NSS pin is managed by the hardware. The NSS signal
	   is driven low as soon as the SPI is enabled in master mode (SPE=1), and is kept
	   low until the SPI is disabled (SPE =0)
	 */
	SPI_SSOEConfig(SPI2,ENABLE);
	//first let's configure the SPI_CR1 register
	uint32_t tempreg = 0;
	//1. configure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode<<MSTR;
	//2. configure the bus config

	if(pSPIHandle->SPIConfig.SPI_BusConfig == FULL_DUPLEX)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == HALF_DUPLEX)
	{
		//bidi mode should be set
		tempreg |= (1<<BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SIMPLEX_RX_ONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<BIDIMODE);
		//rxonly bit must be set
		tempreg |= (1<<RXONLY);
	}
	//3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << BR;

	//4. Configure the DFF (data format)
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << DFF;

	//5. Configure the SSM (softwre slave management)
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SSM;

	//6. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << CPOL;

	//7. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << CPHA;

	//8. Configure SSI to 1 to prevent MODF error... (look at the datasheet about MODF, somewhere says that in master mode, if SSM and SSI is not 1, you get MODF error
	tempreg |= pSPIHandle->SPIConfig.SPI_SSI << SSI;

	pSPIHandle->pSPIx->CR1 = tempreg;

}
/*
 * Function name : SPI_SendData
 *
 * Note!!!!-> This is a blocking call which means that when this func. is called, it doesn't
 * go out the function until the last bit is sent !!!!!
 */

void SPI_SendData(SPI_Struct *pSPIx, uint8_t *pTxBuffer, uint32_t size )
{
	pSPIx->CR1 |= (1<<SPE); //enable SPI

	while(size > 0)
	{
		//1. wait until TXE is set
		while( !(pSPIx->SR & (1<<TXE)) );

		//2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1<<DFF))
		{
			// 16 bit DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			size--;
			size--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 bit dff
			pSPIx->DR = *(pTxBuffer);
			size--;
			pTxBuffer++;
		}
	}

	//before disabling, let's check if SPI is busy or not. If it's not busy, we can close the comunication
	while(pSPIx->SR & (1<<BSY));
	pSPIx->CR1 &= ~(1<<SPE);  //disable SPI
}

void SPI_ReceiveData(SPI_Struct *pSPIx, uint8_t *pRxBuffer, uint32_t size )
{
	pSPIx->CR1 |= (1<<SPE); //enable SPI

	while(size>0)
	{
		while( !(pSPIx->SR & 1));  //check weather the RXNE bit is set in the status register

		if(pSPIx->CR1 & (1<<DFF))
		{
			//16 bit data format
			*((uint16_t*)pRxBuffer++) = pSPIx->DR;
			size -= 2; //we decrease 2 bytes cuz we receive 2 bytes at once
		}
		else
		{
			//8 bit data frame
			*(pRxBuffer++) = pSPIx->DR;
			size--;
		}
	}

	//before disabling, let's check if SPI is busy or not. If it's not busy, we can close the communication
	while(pSPIx->SR & (1<<BSY));
	pSPIx->CR1 &= ~(1<<SPE);  //disable SPI

}

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t Enable)
{
	if(Enable == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISR0 register
				NVIC->ISER[0] |= (1<<IRQNumber);

			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ISR1 register
				NVIC->ISER[1] |= (1<<(IRQNumber-32));

			}
			else if(IRQNumber >=64 && IRQNumber <= 96)
			{
				//program ISR2 register
				NVIC->ISER[2] |= (1<<(IRQNumber-64));
			}


		}
		else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				NVIC->ICER[0] |= (1<<IRQNumber);
			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//ICER1
				NVIC->ICER[1] |= (1<<(IRQNumber-32));

			}
			else if(IRQNumber >=64 && IRQNumber <= 96)
			{
				//ICER2
				NVIC->ICER[2] |= (1<<(IRQNumber-64));
			}
		}
}
void SPI_IRQPriorityConfig(IRQn_Type IRQNumber, uint32_t IRQPriority)
{
	//NVIC->IP[IRQNumber] |= (uint8_t)IRQPriority;
	NVIC->IP[(uint32_t)(IRQNumber)]               = (uint8_t)((IRQPriority << 4U) & (uint32_t)0xFFUL);
}


uint8_t SPI_SendDataIRQ(SPI_Handle *pSPIHandle, uint8_t *pTxBuffer, uint32_t size )
{

	uint8_t state = pSPIHandle->TxState;

	if(state == SPI_READY)
	{
		//1. save the Tx buffer address and size information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = size;
		//2. mark the spi state as busy in transmission so that no other code can take over the same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3. enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1<<TXEIE);
	}

	return state;

}

uint8_t SPI_ReceiveDataIRQ(SPI_Handle *pSPIHandle, uint8_t *pRxBuffer, uint32_t size )
{

	uint8_t state = pSPIHandle->RxState;

	if(state == SPI_READY)
	{
		//1. save the Rx buffer address and size information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = size;
		//2. mark the spi state as busy in transmission so that no other code can take over the same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3. enable the TXEIE control bit to get interrupt whenever RXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1<<RXNEIE);
	}

	return state;

}

void SPI_IRQHandling(SPI_Handle *pHandle)
{
	uint8_t temp1,temp2;
	//first let's check for TXE
	temp1 = pHandle->pSPIx->SR & (1<<TXE);
	temp2 = pHandle->pSPIx->CR2 & (1<<TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}
	//check for RXNE
	temp1 = pHandle->pSPIx->SR & (1<<RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1<<RXNEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_rxne_interrupt_handle(pHandle);
	}
	//check for overrun error
	temp1 = pHandle->pSPIx->SR & (1<<OVR);
	temp2 = pHandle->pSPIx->CR2 & (1<<ERRIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_ovr_interrupt_handle(pHandle);
	}
}

static void spi_txe_interrupt_handle(SPI_Handle *pHandle)
{
	pHandle->pSPIx->CR1 |= (1<<SPE); //enable SPI

	if(pHandle->pSPIx->CR1 & (1<<DFF))
	{
		// 16 bit DFF
		pHandle->pSPIx->DR = *((uint16_t*)pHandle->pTxBuffer);
		pHandle->TxLen--;
		pHandle->TxLen--;
		(uint16_t*)pHandle->pTxBuffer++;
	}
	else
	{
		// 8 bit dff
		pHandle->pSPIx->DR = *(pHandle->pTxBuffer);
		pHandle->TxLen--;
		pHandle->pTxBuffer++;
	}
	if(! pHandle->TxLen)
	{
		//Txlen is zero, so close the spi transmission and inform the application that Tx is over
		//this prevents interrupts from setting up of TXE flag
		pHandle->pSPIx->CR2 &= ~(1<<TXEIE);
		pHandle->pTxBuffer = NULL;
		pHandle->TxLen = 0;
		pHandle->TxState = SPI_READY;
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX_CMPLT);

	}


}


static void spi_rxne_interrupt_handle(SPI_Handle *pHandle)
{


	if(pHandle->pSPIx->CR1 & (1<<DFF))
	{
		//16 bit data format
		*((uint16_t*)pHandle->pRxBuffer++) = pHandle->pSPIx->DR;
		pHandle->RxLen -= 2; //we decrease 2 bytes cuz we receive 2 bytes at once
	}
	else
	{
		//8 bit data frame
		*(pHandle->pRxBuffer++) = pHandle->pSPIx->DR;
		pHandle->RxLen--;
	}

	if(! pHandle->RxLen)
	{
		//Txlen is zero, so close the spi transmission and inform the application that Tx is over
		//this prevents interrupts from setting up of TXE flag
		pHandle->pSPIx->CR2 &= ~(1<<RXNEIE);
		pHandle->pRxBuffer = NULL;
		pHandle->RxLen = 0;
		pHandle->RxState = SPI_READY;
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_RX_CMPLT);
	}



}

static void spi_ovr_interrupt_handle(SPI_Handle *pHandle)
{

}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle *pHandle, uint8_t AppEv)
{
	//this is a weak implementation. the user application may override this funtion
}
