/******************************************************************************
spitest.cpp
Raspberry Pi SPI interface demo
Byron Jacquot @ SparkFun Electronics>
4/2/2014
https://github.com/sparkfun/Pi_Wedge

A brief demonstration of the Raspberry Pi SPI interface, using the SparkFun
Pi Wedge breakout board.

Resources:

This example makes use of the Wiring Pi library, which streamlines the interface
to the the I/O pins on the Raspberry Pi, providing an API that is similar to the
Arduino.  You can learn about installing Wiring Pi here:
https://github.com/WiringPi/WiringPi/releases

The wiringPi SPI API is documented here:
https://projects.drogon.net/raspberry-pi/wiringpi/spi-library/

The init call returns a standard file descriptor.  More detailed configuration
of the interface can be performed using ioctl calls on that descriptor.
See the wiringPi SPI implementation (wiringPi/wiringPiSPI.c) for some examples.
Parameters configurable with ioctl are documented here:
http://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/spi/spidev

Hardware connections:

This file interfaces with the SparkFun Serial 7 Segment display:
https://www.sparkfun.com/products/11629

The board was connected as follows:
(Raspberry Pi)(Serial 7 Segment)
GND  -> GND
3.3V -> Vcc
CE1  -> SS (Shift Select)
SCK  -> SCK 
MOSI -> SDI
MISO -> SDO

To build this file, I use the command:
>  g++ spitest.cpp -lwiringPi

Then to run it, first the spi kernel module needs to be loaded.  This can be 
done using the GPIO utility.
> gpio load spi
> ./a.out

This test uses the single-segment mode of the 7 segment display.  It shifts a 
bit through the display characters, lighting a single character of each at a 
time.

Development environment specifics:
Tested on Raspberry Pi V2 hardware, running Raspbian.
Building with GCC 4.6.3 (Debian 4.6.3-14+rpi1)

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPiSPI.h>
#include <unistd.h>

// channel is the wiringPi name for the chip select (or chip enable) pin.
// Set this to 0 or 1, depending on how it's connected.
static const int CHANNEL = 0;
unsigned char CONFIG_REG = 0x00;
unsigned char WRITE_REG_NRF = 0x20; //write command for NRF24L01+
unsigned char READ_REG_NRF = 0x00; //read command for NRF24L01+

unsigned char ENAA = 0x01;
unsigned char SETUP_AW = 0x03;
unsigned char RF_SETUP = 0x06;
unsigned char RX_ADDR_P01 = 0x0A;
unsigned char RX_PW_P0 = 0x11;
unsigned char TX_ADDR = 0x10;
unsigned char WRITE_PAYLOAD_NRF = 0xA0; //write TX FIFO command for NRF24L01+
unsigned char READ_PAYLOAD_NRF = 0x60;

int main()
{
    int fd, result;
    unsigned char buffer[1] = {1}; //is this the right size?

    printf("Initializing\n");

    // Configure the interface.
    // CHANNEL insicates chip select,
    // 500000 indicates bus speed.
    fd = wiringPiSPISetup(CHANNEL, 500000); 

    printf("Init result: \n");
    printf("%d\n", fd);

    int len = sizeof(buffer)/sizeof(buffer[0]);
    pinMode(8, OUTPUT); //set CE pin to output //WHICH PIN IS CE?
    pinMode(9, INPUT); //set IRQ pin to output //WHICH PIN IS IRQ?

    delay_microseconds(1000*100); //give the chip time to power up
    receiveByteNRF(buffer);

    printf("Data: \n");
    printf("%d\n", buffer[0]);
}

/**
 * @brief reads from the SPI bus
 * @param reg_addr: Address of the register where data is going to be read from
 * @param buffer: Pointer to data buffer that stores the data read
 * @param len: Number of bytes of data to be read
 */
void readwriteNRF_SPI(unsigned char reg_addr, unsigned char * buffer, int len, unsigned char command){
    unsigned char new_buffer[len+1];
	int result;
	
	new_buffer[0] = command | reg_addr; 
	
	// Copy source array to destination array with offset
    // memcpy(new_buffer + 1, buffer, len * sizeof(unsigned char));
	
    // Copy elements from array1 to array2 starting at index 1
    memcpy(&new_buffer[1], buffer, len * sizeof(unsigned char));

    // Copy elements from array1 to array2 starting at index 1
    /*
    for (int i = 0; i < len; i++) {
        new_buffer[i + 1] = buffer[i];
    }*/

	result = wiringPiSPIxDataRW(0, CHANNEL, new_buffer, len+1);
	//result is unused at present
}

/** 
* @brief: transmits a byte of data for testing purposes
* @param: data byte of data to be transmitted
*/
 //TODO make this much more functional
void receiveByteNRF(unsigned char data){
    unsigned char buffer[1] = {0}; 
    unsigned char addressWidth = 0x01; // Variable to hold the address width
    unsigned char rfSetup = 0x20; // Variable to hold the RF setup value
    unsigned char configPRX = 0x0B; // Variable to hold the PRX mode config
    unsigned char configPowerDown = 0x09; // Variable to hold the power down config
    unsigned long rxAddress = 0x93BD6B; // Variable to hold the RX address

    //set control registers
    readwriteNRF_SPI(SETUP_AW, &addressWidth, 1, WRITE_REG_NRF); //set to 3 byte address width
    readwriteNRF_SPI(RX_ADDR_P01, (unsigned char*)&rxAddress, 1, WRITE_REG_NRF); //set write address
    readwriteNRF_SPI(RF_SETUP, &rfSetup, 1, WRITE_REG_NRF); //set RF Data Rate to 250kbps, RF output power to -18dBm
    //write data to be transmitted into TX FIFO
    readwriteNRF_SPI(0x00, &data, 1, WRITE_PAYLOAD_NRF);
    readwriteNRF_SPI(CONFIG_REG, &configPRX, 1, WRITE_REG_NRF); //set to PRX mode

    digitalWrite(8, HIGH); //enable chip to receive data
    delay_microseconds(130);
    Delay(1);
    while(digitalRead(9)){ //wait for data to be received (IRQ pin is active low)
        delay_microseconds(100*1000);  //TODO add better delay function with millisecond precision
    }          
    digitalWrite(9, HIGH); //undo interrupt signal

    readwriteNRF_SPI(0x00, buffer, 1, READ_PAYLOAD_NRF); //read data from RX FIFO
    digitalWrite(8, LOW); //switch chip to standby mode by disabling CE pin

    printf("Data received: %d\n", buffer[0]);
    set_nrf24_SPI_CE(0); //disable chip after reception
    readwriteNRF_SPI(CONFIG_REG, &configPowerDown, 1, WRITE_REG_NRF); //power down by writing to config register
}

/**
 * @brief  Delay function 
 * @param  usec: specifies the delay time length, in 1 microsecond.
 * @retval None
 */
void __attribute__((optimize("O0"))) delay_microseconds(unsigned int usec){
  for(volatile unsigned int counter = 0; counter < usec; counter++){
    //do nothing NOP instructions
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();
    //lol this is nearly perfect timing
  }
}





