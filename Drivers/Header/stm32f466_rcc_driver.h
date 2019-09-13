/*
 * stm32f466_rcc_driver.h
 *
 *  Created on: Sep 6, 2019
 *      Author: Simon
 */

#ifndef INC_STM32F466_RCC_DRIVER_H_
#define INC_STM32F466_RCC_DRIVER_H_

/*
 * RCC_GetPCLK1Value will return the clock frequency for APB1 bus.
 * In order to find out the APB1 bus frequency, you need to find out the system clock used (ex. HSI, HSE, PLL.)
 * There are two prescalers on the way to APB1 bus and you need divide the Sysclock twice in order to find the clock for the APB1 bus.
 */
//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F466_RCC_DRIVER_H_ */
