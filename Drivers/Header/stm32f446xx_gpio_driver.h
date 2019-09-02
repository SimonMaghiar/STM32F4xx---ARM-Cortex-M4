/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Aug 23, 2019
 *      Author: Simon
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_


#include "stm32f466xx.h"



typedef struct
{
	uint8_t GPIO_PinNUmber;			/* possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;			/* possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/* possible values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;	/* GPIOx_PUPDR possible values from @GPIO_PIN_OUT_PuPd */
	uint8_t GPIO_PinOPType;			/* GPIOx_OTYPER */
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig;

/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_Struct *pGPIOx;			/* This hold the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig GPIO_PinConfig;	/* This holds GPIO pin configuration settings */

}GPIO_Handle;



/*
 * @GPIO_PIN_NUMBERS
 * PIN NUMBERS
 */

#define GPIO_PIN_0	0
#define GPIO_PIN_1	1
#define GPIO_PIN_2	2
#define GPIO_PIN_3	3
#define GPIO_PIN_4	4
#define GPIO_PIN_5	5
#define GPIO_PIN_6	6
#define GPIO_PIN_7	7
#define GPIO_PIN_8	8
#define GPIO_PIN_9	9
#define GPIO_PIN_10	10
#define GPIO_PIN_11	11
#define GPIO_PIN_12	12
#define GPIO_PIN_13	13
#define GPIO_PIN_14	14
#define GPIO_PIN_15	15
/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT	 	4		//input, falling edge intrerrupt trigger
#define GPIO_MODE_IT_RT	 	5		//input, rising edge
#define GPIO_MODE_IT_RFT	6		// rising and falling edge

/*
 * GPIO output types
 */
#define GPIO_OUT_PUSHPULL	0
#define GPIO_OUT_OPENDRAIN	1

/*
 * @GPIO_PIN_SPEED
 * GPIO possible output speeds
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HSPEED	3

/*
 * @GPIO_PIN_OUT_PuPd
 * GPIO port pull-up/pull-down registers
 */

#define GPIO_NO_PU_PD	0
#define GPIO_PU			1
#define GPIO_PD			2


/**********************************************************************************************
 * 											APIs supported by this driver
 * 							For more informations about the APIs chech the function definitions
 *********************************************************************************************** */


/* Peripheral Clock Setup */
void GPIO_PeriClockControl(GPIO_Struct *pGPIOx, uint8_t Enable);

/* Init and De-Initialization */
void GPIO_Init(GPIO_Handle *pGPIOHandle);
void GPIO_DeInit(GPIO_Struct *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Struct *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Struct *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_Struct *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_Struct *pGPIOx, uint16_t value);

void GPIO_ToggleOutPin(GPIO_Struct *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t Enable);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(IRQn_Type IRQNumber, uint32_t IRQPriority);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
