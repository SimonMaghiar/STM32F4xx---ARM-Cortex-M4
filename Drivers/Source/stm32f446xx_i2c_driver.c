/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Sep 2, 2019
 *      Author: Simon
 */

#include "stm32f466xx.h"
#include "stm32f446_i2c_driver.h"

void I2C_CloseReceiveData(I2C_Handle *pI2CHandle);
void I2C_CloseSendData(I2C_Handle *pI2CHandle);

void I2C_PeriClockControl(I2C_Struct *pI2Cx, uint8_t Enable)
{
	if(Enable == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_CLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_CLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_CLK_EN();
		}
	}
	else
	{

	}
}

uint16_t AHB_presc[8] = {2,4,8,16,64,128,256,512};
uint8_t AHB1_presc[4] = {2,4,8,16};

/*
 * RCC_GetPCLK1Value will return the clock frequency for APB1 bus.
 * In order to find out the APB1 bus frequency, you need to find out the system clock used (ex. HSI, HSE, PLL.)
 * There are two prescalers on the way to APB1 bus and you need divide the Sysclock twice in order to find the clock for the APB1 bus.
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;
	uint8_t clkscr,temp,AHB_prescaler,AHB1_pres,temp2;


	clkscr = ((RCC->CFGR >> 2) & 0x3);
	if(clkscr == 0)
	{
		//system clock = HSI (16Mhz)
		SystemClk = 16000000;
	}
	else if(clkscr == 1)
	{
		SystemClk = 8000000;
	}
	else if(clkscr == 2)
	{
		//System clock is using PLL so we need to calculate the frequency
		//code is not implemented for this case
	}
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		AHB_prescaler = 1;
	}
	else
	{
		AHB_prescaler = AHB_presc[temp-8];
	}

	temp2 = ((RCC->CFGR >> 10) & 0x7);
	if(temp2 < 4)
	{
		AHB1_pres = 1;
	}
	else
	{
		AHB1_pres = AHB1_presc[temp2-4];
	}
	pclk1 = ((SystemClk / AHB_prescaler) / AHB1_pres);
	return pclk1;
}

void I2C_Init(I2C_Handle *pI2CHandle)
{
	//1. enable peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	uint16_t tempreg = 0;

	//2. program the device address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress <<1; //we shift it cuz bit 0 is "don't care" if we are using 7 bit addressing mode
	tempreg |= (1<< 14); //the datasheet says this bit must be kept at 1 in the OAR1 register
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	/*
	 * NOTE ! In order to program the clock freq. of the I2C register, you need to set the following bits:
	 * FREQ[5:0] bits inside of I2C_CR2 register and CCR[11:0] inside of I2C_CCR register
	 */
	//3. configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= (RCC_GetPCLK1Value() / 1000000U);
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//4.CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		pI2CHandle->pI2Cx->CCR &= ~(1<<15);
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		//mode is fast mode
		tempreg = (1<<15);
		tempreg	= (1<< pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			//if duty = 0 then CCR = FPCLk1 / 3 * Fscl  or  CCR = Tscl / 3* TFPCLK1 ! Look at the datasheet
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//5. TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//in Standard mode, the maximum allowed SCL rise time is 1000 ns.
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		//in Fast mode, the maximum allowed SCL rise time is 300 ns.
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

	//enable the peripheral after the bits were stored in the registers
	//NOTE! If you look at the ACK bit, it says that if PE=0 then the ACK bit will be cleared.
	//Enable the peripheral and write to ACK bit !
	pI2CHandle->pI2Cx->CR1 |= (1<<PE);

	//LAST. ackowledge control bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << ACK);
	pI2CHandle->pI2Cx->CR1 = tempreg;


}

uint8_t I2C_FlagStatus(I2C_Struct *pI2Cx,uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return 1;
	}
	return 0;
}

static void clearADDRFlag(I2C_Struct *pI2Cx)
{
	uint16_t dummy_read = pI2Cx->SR1;
	         dummy_read = pI2Cx->SR2;
	(void)dummy_read; //this is to prevent compiler optimization
}

void I2C_MasterSendData(I2C_Handle *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//look at the transfer sequence diagram for master transmitter in datasheet : 7-bit master transmitter
	//1. generate the START condition
	pI2CHandle->pI2Cx->CR1 |= (1<<START);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//NOTE: Until SB is cleared SCL will be strectched (pulled to low)
	while( ! I2C_FlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG) );

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	pI2CHandle->pI2Cx->DR = (SlaveAddr << 1); //the LSB is shifted by one to leave space for the R/W bit. In this case, R/W = 0

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1

	while( ! I2C_FlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG) );

	//5. In order to clear the ADDR flag, you must first read the SR1 register followed by reading SR2 register
	clearADDRFlag(pI2CHandle->pI2Cx);

	//6. Write data is repeated start until all the bytes are sent
	while(Len > 0)
	{
		//check if TxE is 1 which means data register empty
		if(pI2CHandle->pI2Cx->SR1 & I2C_TXE_FLAG)
		{
			pI2CHandle->pI2Cx->DR = *(pTxbuffer++);
			Len--;
		}
	}

	//7. When Len becomes zero, wait for TxE=1 and BTF=1 before generating the STOP condition
	//NOTE: TxE=1 , BTF=1 means that both SR and DR are empty and next transmission should begin
	//when BTF=1 SCL will be stretched

	while( !I2C_FlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG));
	while( !I2C_FlagStatus(pI2CHandle->pI2Cx,I2C_BTF_FLAG));

	//8. generate stop condition
	pI2CHandle->pI2Cx->CR1 |= (1<<STOP);

}

void I2C_MasterReceiveData(I2C_Handle *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	pI2CHandle->pI2Cx->CR1 |= (1<<START);

	//2. clear START BIT by polling it
	while( ! I2C_FlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG) );

	//3. Send the address of the slave we want to read from
	pI2CHandle->pI2Cx->DR = ((SlaveAddr << 1) | 0x1 );  //the last bit will be 1 which means this will be a read operation NOTE!: Slave address is only 7 bit, so 8'th bit is R/W bit

	//4. check if address match and clear ADDR (if set) by reading SR1 and SR2 register
	while( ! I2C_FlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG) );
	clearADDRFlag(pI2CHandle->pI2Cx);

	if(Len == 1)  //if we only want to receive one byte
	{
		//disable Acking
		pI2CHandle->pI2Cx->CR1 &= ~(1<<ACK);

		//generate Stop condition
		pI2CHandle->pI2Cx->CR1 |= (1<<STOP);

		//wait until RXNE becomes 1
		while( !(pI2CHandle->pI2Cx->SR1 & I2C_RXNE_FLAG) );

		//Read the data
		*pRxbuffer =  pI2CHandle->pI2Cx->DR;

		Len--;
	}

	while(Len > 0)	//if we want to receive more bytes than 1
	{
		//check if RXNE flag is set...if RXNE=1 Data register not empty
		if(pI2CHandle->pI2Cx->SR1 & I2C_RXNE_FLAG)
		{
			// read from DR register
			*(pRxbuffer++) = pI2CHandle->pI2Cx->DR;
			Len--;
		}
		if(Len == 1)
		{
			pI2CHandle->pI2Cx->CR1 &= ~(1<<ACK);  //datasheet says after before last byte is sent, dissable ACK bit and trigger stop bit
			pI2CHandle->pI2Cx->CR1 |= (1<<STOP);
		}
	}

	pI2CHandle->pI2Cx->CR1 |= (1<<ACK);  //enable ACK
}


/*
 * NON-BLOCKING APIS
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t Enable)
{
	if(Enable == ENABLE)
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
void I2C_IRQPriorityConfig(IRQn_Type IRQNumber, uint32_t IRQPriority)
{
	NVIC->IP[(uint32_t)(IRQNumber)]= (uint8_t)((IRQPriority << 4U) & (uint32_t)0xFFUL);  //you must shift it with 4 to the left because the first 4 bits are reserved!
}


uint8_t I2C_MasterSendDataIRQ(I2C_Handle *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate START Condition which will trigger an interrupt
		pI2CHandle->pI2Cx->CR1 |= (1<<START);

		//enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << ITBUF);

		//enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << ITEVT);

		//enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << ITERR);

	}

	return busystate;
}
uint8_t I2C_MasterReceiveDataIRQ(I2C_Handle *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //RxSize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1<<START);

		//enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << ITBUF);

		//enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << ITEVT);

		//enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << ITERR);

	}

	return busystate;
}


void I2C_EV_IRQHandling(I2C_Handle *pI2CHandle)
{
	//Interrupt handler for both mater and slave mode of a device

	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1<<ITEVT);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1<<ITBUF);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<SB);
	//1. Handle for interrupt generated by SB event*************************************************************
	//Note: SB flag is only applicable in Master mode

	if(temp1 && temp3)
	{
		//Interrupt is generated because of SB event
		//NOTE!: This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			pI2CHandle->pI2Cx->DR = (pI2CHandle->DevAddr << 1);		//send 7 bit + R/W bit (this case it's 0 )
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			pI2CHandle->pI2Cx->DR = ((pI2CHandle->DevAddr << 1) | 0x1); // R/W bit is 1 because we want to read data from the slave
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<ADDR);
	//2. Handle for interrupt generated by ADDR event ************************************************************
	//Note: When master mode: Address is sent
	//		When slave mode	: Address matched with own address
	if(temp1 && temp3)
	{
		//ADDR flag is set
		clearADDRFlag(pI2CHandle->pI2Cx);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<BTF);
	//3. Handle for interrupt generated by BTF(Byte Transfer Finished) event***************************************
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle->TxLen == 0)
			{
			//make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1<<TxE))
			{
				//BTF and TXE = 1 which indicates that we can stop the transmission
				//1. generate STOP condition
				if(pI2CHandle->Sr == REPEATED_START_DISABLE)
				{
					pI2CHandle->pI2Cx->CR1 |= (1<<STOP);
				}

				//2. reset all the member elemenets of the handle structure.
				I2C_CloseSendData(pI2CHandle);
			}

			}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2CHandle->RxLen == 0)
				{

				}
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<STOP);
	//4. Handle for interrupt generated by STOPF event******************************************************************
	//Note: Stop detection flag is applicable only in slave mode.
	if(temp1 && temp3)
	{
		//STOP flag is set
		//1. clear the STOP flag by reading SR1 and after that Write to CR1 (by datasheet)
		uint8_t dummy_read = pI2CHandle->pI2Cx->SR1;
		(void)dummy_read;  //just because of compilation optimization
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<TxE);
	//5. Handle for interrupt generated by TXE event*********************************************************************
	if(temp1 && temp2 && temp3)
	{
		//TXE flag is set

		//the device must be in master mode
		if(pI2CHandle->pI2Cx->SR2 & 0x1)	//MSL bit in SR2 tells you if the device is in master or slave mode
		{

			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				if(pI2CHandle->TxLen > 0)
				{
					//1. load the data into DR

					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer++);
					//2.Decrement length
					pI2CHandle->TxLen--;
				}
				else
				{
					I2C_CloseSendData(pI2CHandle);
				}
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<RxNE);
	//6. Handle for interrupt generated by RXNE event***********************************************************************
	if(temp1 && temp2 && temp3)
	{
			//device is master
			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				//if we only want to receive 1 byte
				if(pI2CHandle->RxSize == 1)
				{
					//disable Acking
					pI2CHandle->pI2Cx->CR1 &= ~(1<<ACK);

					//generate Stop condition
					pI2CHandle->pI2Cx->CR1 |= (1<<STOP);

					if(pI2CHandle->pI2Cx->SR1 & (1<<RxNE))
					{
						*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
						pI2CHandle->RxLen--;
					}
					I2C_CloseReceiveData(pI2CHandle);
				}
				else if(pI2CHandle->RxSize > 0)
				{
				if(pI2CHandle->RxLen > 0)	//if we want to receive more bytes than 1
					{
						//check if RXNE flag is set...if RXNE=1 Data register not empty
						if(pI2CHandle->pI2Cx->SR1 & I2C_RXNE_FLAG)
						{
							// read from DR register
							*(pI2CHandle->pRxBuffer++) = pI2CHandle->pI2Cx->DR;
							pI2CHandle->RxLen--;
						}
						if(pI2CHandle->RxLen == 0)
						{
							pI2CHandle->pI2Cx->CR1 &= ~(1<<ACK);  //datasheet says after before last byte is sent, dissable ACK bit and trigger stop bit
							pI2CHandle->pI2Cx->CR1 |= (1<<STOP);
							I2C_CloseReceiveData(pI2CHandle);
						}
					}
				}
			}
	}
}











//Error handling
void I2C_ER_IRQHandling(I2C_Handle *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << ITERR);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << BERR);

		//Implement the code to notify the application about the error
	   //I2C_ApplicationEventCallback(pI2CHandle,BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag

		//Implement the code to notify the application about the error

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag

		//Implement the code to notify the application about the error
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag

		//Implement the code to notify the application about the error
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag

		//Implement the code to notify the application about the error
	}

}



//CLOSE CONNECTION ****************************

void I2C_CloseReceiveData(I2C_Handle *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << ITBUF);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << ITEVT);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		pI2CHandle->pI2Cx->CR1 |= (1<<ACK);
	}
}

void I2C_CloseSendData(I2C_Handle *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << ITBUF);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << ITEVT);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

//SLAVE IMPLEMENTATION**********************************************************************************

void I2C_SlaveSendData(I2C_Struct *pI2C,uint8_t data)
{
	pI2C->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_Struct *pI2C)
{
	return (uint8_t)pI2C->DR;
}

