/*
 * spi_Tx_only_arduino.c
 *
 *  Created on: Aug 30, 2019
 *      Author: Simon
 */

/*
 * spi_tx_testing.c
 *
 *  Created on: Aug 30, 2019
 *      Author: Simon
 */

#include "stm32f466xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f466_spi_driver.h"
#include "string.h"

//command codes

#define COMMAND_LED_CTRL	0x50
#define COMMAND_SENSOR_READ	0x51
#define COMMAND_LED_READ	0x52
#define COMMAND_PRINT		0x53
#define COMMAND_ID_READ		0x54

#define LED_ON	1
#define LED_OFF	0


//arduino external led connected

#define LED_PIN		9

SPI_Handle SPI2handle;

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 *
 * ALT function mode: AF5
 */
void SPI2_GPIOInits(void)
{
	//SCLK
	GPIO_Handle gpio;
	gpio.pGPIOx = GPIOB;
	gpio.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_13;
	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_PUSHPULL;
	gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;		// we use pull-up for each pin because there's a slave connected to it!!!!!!
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&gpio);

	//MOSI
	gpio.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_15;
	GPIO_Init(&gpio);

	//MISO
	gpio.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_14;
	GPIO_Init(&gpio);

	//NSS
	gpio.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_12;
	GPIO_Init(&gpio);

}
void SPI2_Inits(void)
{

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_BusConfig = FULL_DUPLEX;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;  //generates clock of 8Mhz
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;		//here we use hardware slave management so the NSS pin will be pulled to low automaticlly when the SPI is enabled!
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	// this makes NSS signal internally high and avoids MODF error but we use hardware management so we can comment it out
	//spi.SPIConfig.SPI_SSI = SPI_SSI_HIGH;

	SPI_Init(&SPI2handle);
}

void GPIO_BUTTONInit()
{
	GPIO_Handle GpioButton;
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;

	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GpioButton);
}

void delay()
{
	for(uint32_t i =0; i<500000;i++);
}


void SPI2_IRQHandler(void)
{

	SPI_IRQHandling(&SPI2handle); //SPI2Handle is declared globaly !
}


int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read = 0xff;
	SPI2_GPIOInits();
	SPI2_Inits();
	GPIO_BUTTONInit();
	SPI_IRQConfig(SPI2_IRQn,ENABLE);
	SPI2->CR1 |= (1<<SPE); //enable SPI from the beginning
	while(1)
	{
		if(!GPIO_ReadFromInputPin(GPIOC,13))
		{
			delay();
			//first let's send the length of the data we are going to send

			//1. CMD_LED_CTRL
			uint8_t	command = COMMAND_LED_CTRL;
			uint8_t ackbyte;
			uint8_t args[2];
			SPI_SendDataIRQ(&SPI2handle,&command,1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveDataIRQ(&SPI2handle,&dummy_read,1);

			//send some dummy bits (1byte) to fetch the response from the slave !
			SPI_SendDataIRQ(&SPI2handle,&dummy_write,1);

			//read the ack byte received
			SPI_ReceiveDataIRQ(&SPI2handle,&ackbyte,1);

			if(ackbyte == 0xF5)
			{
				//the slave acknowledged the command
				args[0] = LED_PIN;
				args[1] = LED_ON;
				SPI_SendDataIRQ(&SPI2handle,args,2);
			}

		}
	}


	return 0;
}
