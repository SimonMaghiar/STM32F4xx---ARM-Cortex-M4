/*
 * spi_tx_testing.c
 *
 *  Created on: Aug 27, 2019
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
	gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&gpio);

	//MOSI
	gpio.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_15;
	GPIO_Init(&gpio);

	//MISO
	//gpio.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_14;
	//GPIO_Init(&gpio);

	//NSS
	//gpio.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_12;
	//GPIO_Init(&gpio);

}
void SPI2_Inits(void)
{
	SPI_Handle spi;

	spi.pSPIx = SPI2;
	spi.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi.SPIConfig.SPI_BusConfig = FULL_DUPLEX;
	spi.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	spi.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;  //generates clock of 8Mhz
	spi.SPIConfig.SPI_SSM = SPI_SSM_EN;
	spi.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	spi.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
// you need to add this too otherwise you get modf status register 1
	spi.SPIConfig.SPI_SSI = SPI_SSI_HIGH;

	SPI_Init(&spi);
}

int main(void)
{
	SPI2_GPIOInits();
	SPI2_Inits();
	char user_data[] = "Hello world";
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));
	return 0;
}
