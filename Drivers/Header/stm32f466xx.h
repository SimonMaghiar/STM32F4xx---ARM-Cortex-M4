/*
 * stm32f466xx.h
 *
 *  Created on: Aug 23, 2019
 *      Author: Simon
 */

#ifndef INC_STM32F466XX_H_
#define INC_STM32F466XX_H_

#include <stdint.h>
#include <stddef.h>

#define _vo	volatile

#define FLASH_BASEADDR	0x08000000UL   /* Base address of flash memory */
#define SRAM1_BASEADDR	0x20000000UL   /* Base address of SRAM1 */
#define ROM_BASEADDR	0x1FFF0000UL   /* Base address of System Memory a.k.a ROM */
#define SRAM	SRAM1_BASEADDR

#define RCC_BASEADDR	0x40023800UL  /* Base address of RCC register */


/*
 * AHBx and APBx Bus Peripheral Base Addresses
 */

#define PERIPH_BASE		0x40000000UL
#define APB1_BASE_ADDR	PERIPH_BASE
#define APB2_BASE_ADDR	0x40010000UL
#define AHB1_BASE_ADDR	0x40020000UL
#define AHB2_BASE_ADDR	0x50000000UL


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR		(AHB1_BASE_ADDR + 0x0000UL)
#define GPIOB_BASEADDR		(AHB1_BASE_ADDR + 0x0400UL)
#define GPIOC_BASEADDR		(AHB1_BASE_ADDR + 0x0800UL)
#define GPIOD_BASEADDR		(AHB1_BASE_ADDR + 0x0C00UL)
#define GPIOE_BASEADDR		(AHB1_BASE_ADDR + 0x1000UL)
#define GPIOF_BASEADDR		(AHB1_BASE_ADDR + 0x1400UL)
#define GPIOG_BASEADDR		(AHB1_BASE_ADDR + 0x1800UL)
#define GPIOH_BASEADDR		(AHB1_BASE_ADDR + 0x1C00UL)

/*
 * Base addresses of peripherals which are hanging on APB1 bus (only the one we are interested in)
 */

#define I2C1_BASEADDR		(APB1_BASE_ADDR + 0x5400)
#define I2C2_BASEADDR		(APB1_BASE_ADDR + 0x5800)
#define I2C3_BASEADDR		(APB1_BASE_ADDR + 0x5C00)

#define SPI2_BASEADDR		(APB1_BASE_ADDR + 0x3800)
#define SPI3_BASEADDR		(APB1_BASE_ADDR + 0x3C00)

#define USART2_BASEADDR		(APB1_BASE_ADDR + 0x4400)
#define USART3_BASEADDR		(APB1_BASE_ADDR + 0x4800)

#define UART4_BASEADDR		(APB1_BASE_ADDR + 0x4C00)
#define UART5_BASEADDR		(APB1_BASE_ADDR + 0x5000)


/*
 * Base addresses of peripherals which are hanging on APB2 bus (only the one we are interested in)
 */

#define SPI1_BASEADDR		(APB2_BASE_ADDR + 0x3000)

#define USART1_BASEADDR		(APB2_BASE_ADDR + 0x1000)
#define USART6_BASEADDR		(APB2_BASE_ADDR + 0x1400)

#define EXTI_BASEADDR		(APB2_BASE_ADDR + 0x3C00)

#define SYSCFG_BASEADDR		(APB2_BASE_ADDR + 0x3800)

#define NVIC_BASEADDR		(0xE000E100)


/*
 * Structure of the peripherals to access the registers of ** GPIO **
 */


typedef struct
{
	_vo uint32_t MOD;		/* Mode register offset: 0x00*/
	_vo uint32_t OTYPER;	/* GPIO Port output type register: 0x04 */
	_vo uint32_t OSPEEDR;	/* GPIO Port output speed register: 0x08 */
	_vo uint32_t PUPDR;		/* GPIO Port pull-up/pull-down register: 0x0C */
	_vo uint32_t IDR;		/* GPIO Port input data register: 0x10 */
	_vo uint32_t ODR;		/* GPIO Port output data register: 0x14 */
	_vo uint32_t BSRR;		/* GPIO Port bit set/reset register: 0x18 */
	_vo uint32_t LCKR;		/* GPIO Port configuration lock register: 0x1C */
	_vo uint32_t AFR[2];	/* Alternate functions ->> AFR[0]:Low registers, AFR[1] HIGH registers */
}GPIO_Struct;

#define GPIOA	((GPIO_Struct*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_Struct*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_Struct*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_Struct*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_Struct*)GPIOE_BASEADDR)
#define GPIOF	((GPIO_Struct*)GPIOF_BASEADDR)
#define GPIOG	((GPIO_Struct*)GPIOG_BASEADDR)
#define GPIOH	((GPIO_Struct*)GPIOH_BASEADDR)

/*
 * Structure register to access ** RCC ** registers
 */

typedef struct
{
	_vo uint32_t CR;			/* 0x00 */
	_vo uint32_t PLL_CFGR;  	/* 0x04 */
	_vo uint32_t CFGR;			/* 0x08 */
	_vo uint32_t CIR;			/* 0x0C */
	_vo uint32_t AHB1RSTR;		/* 0x10 */
	_vo uint32_t AHB2RSTR;		/* 0x14 */
	_vo uint32_t AHB3RSTR;		/* 0x18 */
	uint32_t reserved1;		/* */
	_vo uint32_t APB1RSTR;		/* 0x20 */
	_vo uint32_t APB2RSTR;		/* 0x24 */
	uint32_t reserved2[2];	/* */
	_vo uint32_t AHB1ENR;		/* 0x30 */
	_vo uint32_t AHB2ENR;		/* 0x34 */
	_vo uint32_t AHB3ENR;		/* 0x38 */
	uint32_t reserved3;		/* */
	_vo uint32_t APB1ENR;		/* 0x40*/
	_vo uint32_t APB2ENR;		/* 0x44 */
	uint32_t reserved4[2];	/* */
	_vo uint32_t AHB1LPENR;		/* 0x50 */
	_vo uint32_t AHB2LPENR;		/* 0x54 */
	_vo uint32_t AHB3LPENR;		/* 0x58 */
	uint32_t reserved5;		/* */
	_vo uint32_t APB1LPENR;		/* 0x60 */
	_vo uint32_t APB2LPENR;		/* 0x64 */
	uint32_t reserved6[2];	/* */
	_vo uint32_t BDCR;			/* 0x70 */
	_vo uint32_t CSR;			/* 0x74 */
	uint32_t reserved7[2];	/* */
	_vo uint32_t SSCGR;			/* 0x80 */
	_vo uint32_t PLLI2SCFGR;	/* 0x84 */
	_vo uint32_t PLLSAICFGR;	/* 0x88 */
	_vo uint32_t DCKCFGR;		/* 0x8C */
	_vo uint32_t CKGATENR;		/* 0x90 */
	_vo uint32_t DCKCFGR2;		/* 0x94 */

}RCC_Struct;

#define RCC ((RCC_Struct*)RCC_BASEADDR)


/*
 *  External interrupt structure
 */

typedef struct
{
	_vo uint32_t IMR;	/* Interrupt mask register */
	_vo uint32_t EMR;	/* Event mask register */
	_vo uint32_t RTSR;	/* Rising trigger selection register */
	_vo uint32_t FTSR;	/* Falling trigger selection register */
	_vo uint32_t SWIER;	/* Software interrupt event register */
	_vo uint32_t PR;	/* Pending register */
}EXTI_Struct;

#define EXTI ((EXTI_Struct*)EXTI_BASEADDR)

/*
 *  System configuration structure
 */

typedef struct
{
	_vo uint32_t MEMRMP;	/* Memory remap register */
	_vo uint32_t PMC;		/* Peripheral mode config register*/
	_vo uint32_t EXTICR[4];
	_vo uint32_t RESERVED1[2];	/* */
	_vo uint32_t CMPCR;		/* Compensation cell register*/
	_vo uint32_t RESERVED2[2];	/* */
	_vo uint32_t CFGR;		/* Configuration register */

}SYSCFG_Struct;

#define SYSCFG ((SYSCFG_Struct*)SYSCFG_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_CLK_ENABLE()	( RCC->AHB1ENR |= (1<<0) )
#define GPIOB_CLK_ENABLE()	( RCC->AHB1ENR |= (1<<1) )
#define GPIOC_CLK_ENABLE()	( RCC->AHB1ENR |= (1<<2) ) /* Enable clock for PORT C */

#define GPIOA_CLK_DISABLE()	( RCC->AHB1ENR &= ~(1<<0) )
#define GPIOB_CLK_DISABLE()	( RCC->AHB1ENR &= ~(1<<1) )
#define GPIOC_CLK_DISABLE()	( RCC->AHB1ENR &= ~(1<<2) )



/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_CLK_EN()	( RCC->APB2ENR |= (1<<12) )
#define SPI2_CLK_EN()	( RCC->APB1ENR |= (1<<14) )
#define SPI3_CLK_EN()	( RCC->APB1ENR |= (1<<15) )
#define SPI4_CLK_EN()	( RCC->APB2ENR |= (1<<13) )



/*
 * Clock Enable Macros for SYSCFG
 */
#define SYSCFG_CLK_EN()	(RCC->APB2ENR |= (1<<14) )

#define GPIO_BASEADDR_TO_CODE(x) (	(x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
								    (x == GPIOF) ? 5 :\
								    (x == GPIOG) ? 6 : 0 )
/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0) //This is a technique in C programming to execute multiple C statements using singe C macro
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)

/*********************************** NVIC ******************************************
 * NVIC Struct (Nested Vectored INterrupt Controller
 */

typedef struct
{
  _vo uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
  _vo uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RESERVED1[24U];
  _vo uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
  _vo uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
  _vo uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
  _vo uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
  _vo  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
} NVIC_Struct;


#define NVIC ((NVIC_Struct*)NVIC_BASEADDR)

/*
 * IRQ(Interrupt request) Numbers of STM32F466x MCU
 */

typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare global interrupt                             */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  SAI2_IRQn                   = 91,     /*!< SAI2 global Interrupt                                             */
  QUADSPI_IRQn                = 92,     /*!< QuadSPI global Interrupt                                          */
  CEC_IRQn                    = 93,     /*!< CEC global Interrupt                                              */
  SPDIF_RX_IRQn               = 94,     /*!< SPDIF-RX global Interrupt                                          */
  FMPI2C1_EV_IRQn             = 95,     /*!< FMPI2C1 Event Interrupt                                           */
  FMPI2C1_ER_IRQn             = 96      /*!< FMPI2C1 Error Interrupt                                           */
} IRQn_Type;


/*****************************************************************
 * SPI Configuration
 */

#define SPI1_BASEADDRESS	0x40013000
#define SPI2_BASEADDRESS	0x40003800
#define SPI3_BASEADDRESS	0x40003C00
#define SPI4_BASEADDRESS	0x40013400

typedef struct
{
	uint32_t CR1;	/*offset: 0x00 -> SPI Control register 1  */
	uint32_t CR2;	/*offset: 0x04 -> SPI Control register 2 */
	uint32_t SR;	/*offset: 0x08 -> SPI Status register */
	uint32_t DR;	/*offset: 0x0C -> SPI Data register */
	uint32_t CRC;	/*offset: 0x10 -> SPI CRC polynomical register */
	uint32_t RXCRCR; /*offset: 0x14 -> SPI RX CRC register */
	uint32_t TXCRCR; /*offset: 0x18 -> SPI TX CRC register */
	uint32_t I2SCFGR; /*offset: 0x1C -> SPI_I2S configuration register */
	uint32_t I2SPR;	/*offset: 0x20 -> SPI_I2S prescaler register */
}SPI_Struct;

#define SPI1 ((SPI_Struct*)SPI1_BASEADDRESS)
#define SPI2 ((SPI_Struct*)SPI2_BASEADDRESS)
#define SPI3 ((SPI_Struct*)SPI3_BASEADDRESS)
#define SPI4 ((SPI_Struct*)SPI4_BASEADDRESS)

#define SPI1_STATUS		*((volatile unsigned long*)(SPI1_BASEADDRESS+0x08))
#define SPI2_STATUS		*((volatile unsigned long*)(SPI2_BASEADDRESS+0x08))
#define SPI3_STATUS		*((volatile unsigned long*)(SPI3_BASEADDRESS+0x08))
#define SPI4_STATUS		*((volatile unsigned long*)(SPI4_BASEADDRESS+0x08))
/*
 * Clock enable macros for SPI peripherals
 */
#define SPI1_Clock_En()		(RCC->APB2ENR |= (1<<12))
#define SPI2_Clock_En()		(RCC->APB1ENR |= (1<<14))
#define SPI3_Clock_En()		(RCC->APB1ENR |= (1<<15))
#define SPI4_Clock_En()		(RCC->APB2ENR |= (1<<13))

/*
 * End of SPI Configuartion **************************************************
 */

/*****************************************************************************
 * 	I2C Configuration
 */


typedef struct
{
	uint16_t CR1;				/* Offset:0x00 - Control Register 1 */
		uint16_t reserved0;
	uint16_t CR2;				/* Offset:0x04 - Control Register 2 */
		uint16_t reserved1;
	uint16_t OAR1;				/* Offset:0x08 - Own address register 1 */
		uint16_t reserved2;
	uint16_t OAR2;				/* Offset:0x0C - Own address register 2 */
		uint16_t reserved3;
	uint16_t DR;				/* Offset:0x10 - Data register */
		uint16_t reserved4;
	uint16_t SR1;				/* Offset:0x14 - Status register 1*/
		uint16_t reserved5;
	uint16_t SR2;				/* Offset:0x18 - Status register 2*/
		uint16_t reserved6;
	uint16_t CCR;				/* Offset:0x1C - Clock control register*/
		uint16_t reserved7;
	uint16_t TRISE;				/* Offset:0x20 - TRISE register*/
		uint16_t reserved8;
	uint16_t FLTR;				/* Offset:0x24 - FLTR register*/
}I2C_Struct;

#define I2C1	((I2C_Struct*)I2C1_BASEADDR)
#define I2C2	((I2C_Struct*)I2C2_BASEADDR)
#define I2C3	((I2C_Struct*)I2C3_BASEADDR)

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_CLK_EN()	( RCC->APB1ENR |= (1<<21) )
#define I2C2_CLK_EN()	( RCC->APB1ENR |= (1<<22) )
#define I2C3_CLK_EN()	( RCC->APB1ENR |= (1<<23) )

/*
 * End of I2C Configuration
 */

/******************************************************************************
 * USART Configuration
 */

typedef struct
{
	uint32_t SR;
	uint32_t DR;
	uint32_t BRR;
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t GTPR;
}USART_Struct;

#define USART1 	((USART_Struct*)USART1_BASEADDR)
#define USART2	((USART_Struct*)USART2_BASEADDR)
#define USART3  ((USART_Struct*)USART3_BASEADDR)
#define UART4	((USART_Struct*)UART4_BASEADDR)
#define UART5	((USART_Struct*)UART5_BASEADDR)
#define USART6	((USART_Struct*)USART6_BASEADDR)

#define USART1_CLK_EN()		(RCC->APB2ENR |= (1<<4))
#define USART6_CLK_EN()		(RCC->APB2ENR |= (1<<5))
#define USART2_CLK_EN()		(RCC->APB1ENR |= (1<<17))
#define USART3_CLK_EN()		(RCC->APB1ENR |= (1<<18))
#define UART4_CLK_EN()		(RCC->APB1ENR |= (1<<19))
#define UART5_CLK_EN()		(RCC->APB1ENR |= (1<<20))

#define ENABLE 	1
#define DISABLE 0
#define SET 	1
#define RESET 	0



#endif /* INC_STM32F466XX_H_ */
