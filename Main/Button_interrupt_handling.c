/*
 * main.c
 *
 *  Created on: Aug 23, 2019
 *      Author: Simon
 */

#include "stm32f466xx.h"
#include "stm32f446xx_gpio_driver.h"


void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(13);
	GPIO_ToggleOutPin(GPIOA,5);
}
void delay()
{
	for(uint32_t i =0;i<500000;i++);
}
int main(void)
{
	/***********************************************************
	 * LED INIT
	 */
	GPIO_Handle GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = 0;	//push-pull mode GPIOx_OTYPER
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = 0; 		//GPIOx_PUPDR Register	-no pull-up, pull-down

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);
	/***********************************************************
	 * Button INIT
	 */

	GPIO_Handle GpioButton;
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;

	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GpioButton);
	GPIO_IRQConfig(EXTI15_10_IRQn,ENABLE);
	GPIO_IRQPriorityConfig(EXTI15_10_IRQn,3);

	while(1)
	{
		/*
		if(GPIO_ReadFromInputPin(GPIOC,13))
		{
			GPIO_WriteToOutputPin(GPIOA,5,RESET);
		}
		else
		{
			GPIO_WriteToOutputPin(GPIOA,5,SET);
		}
		*/

	}
}
