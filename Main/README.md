#These are the different projects I made using the drivers

1. # button_interrupt_handling.c 
Is using an external button attached to PC13 and by pressing it, an External interrupt is generated. EXTI15_10_IRQHandler
is the handler function of the interrupt since PC13 can generate an EXTI13 interrupt configured at falling edge. Inside of the interrupt, a built-in 
led on the board is toggled every time we push the button

2. # spi_tx_texting.c

This source file sends a basing "Hello world" though the SPI.  

3. # spi_Tx_only_arduino

Similar to spi_tx_testing.c, but this time firstly we send the length of the data followed by the data itself. There's an arduino file for receiving 
the length and data and you can visualize it on the Serial monitor. Also, there's a screenshot taken using logic analizer during the transmision
of the data

4. # spi_cmd_handling.c

Full-duplex configuration, transmiting and receiving data from the Arduino Uno. The master (STM32F4xx) sends a command (ex. COMMAND_LED_CTRL = 0x50) 
and the arduino (slave) sends back an acknowledgemnt. If the arduino acknowledged the command, the master will send the pin number and whether the led should be on or off.
There's also a screenshot taken using logic analizer.

5. # spi_cmd_handling_interrupt.c 

The same principle as the 4. but in this case we use interrupt to generate the transmission and recepcion of the data. This is a *non-blocking alternative*.
 
6. # i2c_master_tx_testing.c

By pressing the button, we send data over I2C. We are using standard mode (100Khz). We don't use interrupts for this. There's also a file captured with logic analyzer to check the signals. Note ! The data is sent to an Arduino Uno, and the sketch for it can be found in the arduino folder.

7. # i2c_master_rx_testing.c

STM32F466 is the master and the Arduino Uno is the slave.  The working priciple is the following:
Based on the commands we send to the slave, the slave will respond differently.
1. We send a command_code = (0x51) to Arduino and this command means that we are requesting the length of data that we want to receive
2. We send a read command (R/W bit = 1) to receive the length and store it in the len variable.
3. We send a command_code = (0x52) to slave to ask to start transmitting the data.
4. We send a read command (R/W bit = 1) to receive the data from the slave and store the data in a global array (rcv_buf) 

Note: We use 7 bit addressing mode and the first byte we send after START bit is (Address bit << 1) + (R/W bit) followed by acknowledgemt


This project

