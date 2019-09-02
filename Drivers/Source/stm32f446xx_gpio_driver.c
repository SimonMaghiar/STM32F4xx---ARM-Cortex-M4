/*
 * stm32f446xx_gpio.c
 *
 *  Created on: Aug 23, 2019
 *      Author: Simon
 */


#include "stm32f446xx_gpio_driver.h"


/************************************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This function enables or disables peripherals clock for a given GPIO port
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	- ENABLE or DISABLE macros
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none

*/


void GPIO_PeriClockControl(GPIO_Struct *pGPIOx, uint8_t en)
{
	if(en == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_ENABLE();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_ENABLE();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_ENABLE();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_DISABLE();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_DISABLE();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_DISABLE();
		}
	}
}

/************************************************************
 * @fn			- GPIO_Init
 *
 * @brief		- This function initialize the port
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none

*/

void GPIO_Init(GPIO_Handle *pGPIOHandle)
{
	//0. enable clock peripheral for the particular port
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

	uint32_t temp = 0;
	//1. configure the mode of gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber));
		pGPIOHandle->pGPIOx->MOD &= ~(3<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber); //clear the two bits first before we write to them
		pGPIOHandle->pGPIOx->MOD |= temp;

	}
	else
	{
		//this part will code later (interrupt mode)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1.configure the Falling trigger
			EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
			//clear the corresponding RTSR bit for our safety
			EXTI->RTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure Rising trigger edge
			EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
			//clear the corresponding FTSR bit for our own safety
			EXTI->FTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. both rising and falling edge triggering system
			EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
			EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
		}
		//2. configure the GPIO port selection in SYSCFG_EXTICR

		SYSCFG_CLK_EN(); //enable the clock for syscfg
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber % 4;

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);

		//3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);

	}
	temp = 0;
	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	//3. configure the pupd settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	//4. configure the output type
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber) );
	pGPIOHandle->pGPIOx->OTYPER &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	//5. configre the alt functionality
	if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2 )); //remember ! Before seting bits, first clear them!!!
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2 ));
	}
}

/************************************************************
 * @fn			- GPIO_DeInit
 *
 * @brief		- This function de-initialize the port
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none

*/

void GPIO_DeInit(GPIO_Struct *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{

	}
	else if(pGPIOx == GPIOE)
	{

	}
	else if(pGPIOx == GPIOF)
	{

	}
	else if(pGPIOx == GPIOG)
	{

	}
	else if(pGPIOx == GPIOH)
	{

	}
}
/************************************************************
 * @fn			- GPIO_ReadFromInputPint
 *
 * @brief		- Read the state of an input pin
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	- pin number
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none

*/

uint8_t GPIO_ReadFromInputPin(GPIO_Struct *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;

	return value;
}
/************************************************************
 * @fn			- GPIO_ReadFromInputPort
 *
 * @brief		- Read the state of the port
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none

*/

uint16_t GPIO_ReadFromInputPort(GPIO_Struct *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/************************************************************
 * @fn			- GPIO_WriteToOutputPin
 *
 * @brief		- Write to a specific output pin
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	- pin number
 * @param[in]	- value that can be SET/RESET
 *
 * @return		- none
 *
 * @Note		- none

*/

void GPIO_WriteToOutputPin(GPIO_Struct *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == SET)
	{
		//write 1 to the output data register for the specific pin
		//pGPIOx->BSRR = (1<<PinNumber);
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~(1<<PinNumber);
		//pGPIOx->BSRR = 1<<(2*PinNumber+3);
	}
}

/************************************************************
 * @fn			- GPIO_WriteToOutputPort
 *
 * @brief		- Write to a specific output port
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	- value for which the pins should pe SET or RESET
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none

*/

void GPIO_WriteToOutputPort(GPIO_Struct *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/************************************************************
 * @fn			- GPIO_ToggleOutPin
 *
 * @brief		- Toggle an output pin
 *
 * @param[in]	- base address of the gpio peripheral
 * @param[in]	- pin number that will be toggled
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none

*/

void GPIO_ToggleOutPin(GPIO_Struct *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= 1<<PinNumber;
}

/************************************************************
 * @fn			- GPIO_IRQConfig
 *
 * @brief		- Enable interrupt from the processor side (the NVIC side )
 *
 * @param[in]	- IRQ number of the interrupt (see datasheet)
 * @param[in]	- the priority of the interrupt
 * @param[in]	- enable/disable
 *
 * @return		- none
 *
 * @Note		- none

*/

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t Enable)
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

void GPIO_IRQPriorityConfig(IRQn_Type IRQNumber, uint32_t IRQPriority)
{
	//NVIC->IP[IRQNumber] |= (uint8_t)IRQPriority;
	NVIC->IP[(uint32_t)(IRQNumber)]               = (uint8_t)((IRQPriority << 4U) & (uint32_t)0xFFUL);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1<<PinNumber))
	{
		EXTI->PR |= (1<< PinNumber);
	}
}
