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
	//gpio.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_14;
	//GPIO_Init(&gpio);

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
	SPI2_GPIOInits();
	SPI2_Inits();
	GPIO_BUTTONInit();
	char user_data[] = "Hello world";
	uint8_t datalen = strlen(user_data);
	while(1)
	{
		if(!GPIO_ReadFromInputPin(GPIOC,13))
		{
			delay();
			//first let's send the length of the data we are going to send
			SPI_SendData(SPI2,&datalen,1); //the slave (arduino uno) expects 1 byte
			SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));
		}
	}




	return 0;
}
