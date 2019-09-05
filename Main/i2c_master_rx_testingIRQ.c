/*
 * i2c_master_rx_testing.c
 *
 *  Created on: Sep 3, 2019
 *      Author: Simon
 */

/*
 * i2c_master_tx_testing.c
 *
 *  Created on: Sep 3, 2019
 *      Author: Simon
 */


#include "stm32f466xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446_i2c_driver.h"
#include "string.h"

/*
 * PB8 --> SCL
 * PB9 --> SDA
 *
 * ALT function mode: AF4
 */
#define 	SLAVE_ADDR 		0x68
I2C_Handle I2C1Handle;

//receive buffer
uint8_t rcv_buf[32];

void I2C_GPIOInits(void)
{
	GPIO_Handle	I2CPins;
	I2CPins.pGPIOx = GPIOB;

	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_OPENDRAIN;  //pins should be open-drain with pull-up resister activated
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_8;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNUmber = GPIO_PIN_9;
	GPIO_Init(&I2CPins);

}
void I2C_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61; //this is just a random number but keep in mind that there are some reserved addresses that you can't use ! Search "Reserved addresses" and you will find it
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;  //we don't care about this because we use Standard mode
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;	//Standard mode selected

	I2C_Init(&I2C1Handle);


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

void I2C1_EV_IRQHandler()
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler()
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

int main(void)
{
	I2C_GPIOInits();
	I2C_Inits();
	GPIO_BUTTONInit();

	//I2C IRQ configuration
	I2C_IRQConfig(I2C1_EV_IRQn,ENABLE);
	I2C_IRQConfig(I2C1_ER_IRQn,ENABLE);

	uint8_t command_code;
	uint8_t len;
	//send some data
	while(1)
	{
		if(!GPIO_ReadFromInputPin(GPIOC,13))
		{
			delay();
			command_code = 0x51;

			//send to command to receive the length of data that the arduino will send
			while(I2C_MasterSendDataIRQ(&I2C1Handle, &command_code, 1, SLAVE_ADDR,0) != I2C_READY);
			//store the length in the &len variable
			while(I2C_MasterReceiveDataIRQ(&I2C1Handle, &len, 1, SLAVE_ADDR,0) != I2C_READY);

			command_code = 0x52;
			//send command to receive the data from arduino
			while(I2C_MasterSendDataIRQ(&I2C1Handle, &command_code, 1, SLAVE_ADDR,0) != I2C_READY);

			//send a read request to receive the data and store it in the rcv_buf array
			while(I2C_MasterReceiveDataIRQ(&I2C1Handle, rcv_buf, len, SLAVE_ADDR,0) != I2C_READY);
		}
	}

}
