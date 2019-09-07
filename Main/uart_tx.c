/*
 * uart_tx.c
 *
 *  Created on: Sep 6, 2019
 *      Author: Simon
 */

#include "stm32f466xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446_usart_driver.h"
#include "string.h"

USART_Handle pHandle;

char msg[1024] = "UART Tx testing...\n\r";

void USART2_Init()
{
	pHandle.pUSARTx = USART1;
	pHandle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	pHandle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	pHandle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	pHandle.USART_Config.USART_NrOfStopBits = USART_STOPBITS_1;
	pHandle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	pHandle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&pHandle);
}

void USART2_GPIOInit()
{
	GPIO_Handle usart_gpio;
	usart_gpio.pGPIOx = GPIOA;

	usart_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_PUSHPULL;
	usart_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	usart_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpio.GPIO_PinConfig.GPIO_PinAltFunMode =7;

	//USART2 Tx
	usart_gpio.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_9;
	GPIO_Init(&usart_gpio);

	//USART2 Rx
	usart_gpio.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_10;
	GPIO_Init(&usart_gpio);
}

void GPIO_BUTTONInit()
{
	GPIO_Handle GpioButton;
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;

	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GpioButton);
}
void delay()
{
	for(uint32_t i =0;i<500000;i++);
}

int main(void)
{
	GPIO_BUTTONInit();

	USART2_Init();
	USART2_GPIOInit();

	//enable USART
	pHandle.pUSARTx->CR1 |= (1<<UE);

	while(1)
	{
		if(!GPIO_ReadFromInputPin(GPIOC,13))
		{
			delay();
			USART_SendData(&pHandle,(uint8_t*)msg,strlen(msg));
		}
	}

}
