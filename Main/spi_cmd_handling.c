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

//arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

//arduino external led connected

#define LED_PIN		9


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
	SPI_Handle spi;

	spi.pSPIx = SPI2;
	spi.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi.SPIConfig.SPI_BusConfig = FULL_DUPLEX;
	spi.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	spi.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;  //generates clock of 8Mhz
	spi.SPIConfig.SPI_SSM = SPI_SSM_DI;		//here we use hardware slave management so the NSS pin will be pulled to low automaticlly when the SPI is enabled!
	spi.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	spi.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	// this makes NSS signal internally high and avoids MODF error but we use hardware management so we can comment it out
	//spi.SPIConfig.SPI_SSI = SPI_SSI_HIGH;

	SPI_Init(&spi);
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
	for(uint32_t i =0; i<500000;i++);
}
int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read = 0xff;
	SPI2_GPIOInits();
	SPI2_Inits();
	GPIO_BUTTONInit();

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
			SPI_SendData(SPI2,&command,1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//send some dummy bits (1byte) to fetch the response from the slave !
			SPI_SendData(SPI2,&dummy_write,1);

			//read the ack byte received
			SPI_ReceiveData(SPI2,&ackbyte,1);

			if(ackbyte == 0xF5)
			{
				//the slave acknowledged the command
				args[0] = LED_PIN;
				args[1] = LED_ON;
				SPI_SendData(SPI2,args,2);
			}

		}
	}




	return 0;
}
