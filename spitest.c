#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPiSPI.h>
#include <wiringPi.h>
#include <unistd.h>
#include <time.h>

// channel is the wiringPi name for the chip select (or chip enable) pin.
// Set this to 0 or 1, depending on how it's connected.
static const int CHANNEL = 0;
unsigned char CONFIG_REG = 0x00;
unsigned char WRITE_REG_NRF = 0x20; //write command for NRF24L01+
unsigned char READ_REG_NRF = 0x00; //read command for NRF24L01+

unsigned char ENAA = 0x01;
unsigned char STATUS = 0x07;
unsigned char SETUP_AW = 0x03;
unsigned char RF_SETUP = 0x06;
unsigned char RX_ADDR_P0 = 0x0A;
unsigned char RX_PW_P0 = 0x11;
unsigned char TX_ADDR = 0x10;
unsigned char WRITE_PAYLOAD_NRF = 0xA0; //write TX FIFO command for NRF24L01+
unsigned char READ_PAYLOAD_NRF = 0x60;
unsigned char FLUSH_TX_NRF = 0xE1;
unsigned char FLUSH_RX_NRF = 0xE2;

int main()
{
    int fd, result;
    unsigned char buffer[1] = {1}; //is this the right size?

    printf("Initializing\n");

    // Configure the interface.
    // CHANNEL insicates chip select,
    // 500000 indicates bus speed.
    fd = wiringPiSPISetup(CHANNEL, 500000); 

    printf("Init result: ");
    printf("%d\n", fd);

    int len = sizeof(buffer)/sizeof(buffer[0]);

    // Initialize WiringPi 
    wiringPiSetup();

    pinMode(2, OUTPUT); //set CE pin to output
    pinMode(3, INPUT); //set IRQ pin to input
    digitalWrite(2, LOW); 
    //pullUpDnControl(3, PUD_UP); //enable pull-up resistor on IRQ pin

    delay(200); //give the chip time to power on

    receiveByteNRF();
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

    // Copy elements from array1 to array2 starting at index 1
    memcpy(&new_buffer[1], buffer, len * sizeof(unsigned char));

    result = wiringPiSPIDataRW(CHANNEL, new_buffer, len+1);
    memcpy(buffer, &new_buffer[1], len * sizeof(unsigned char));
	//result is unused at present
}

/**
 * @brief Send command to NRF24L01+ 
 * @param command: Command to be sent to the NRF24L01+
 */
void commandNRF_SPI(unsigned char command){
    unsigned char new_buffer[1];
    int result;
    
    new_buffer[0] = command; 

    result = wiringPiSPIDataRW(CHANNEL, new_buffer, 1); //result is unused at present
}

/** 
* @brief: transmits a byte of data for testing purposes
* @param: data byte of data to be transmitted
*/
 //TODO make this much more functional
void receiveByteNRF(){
    unsigned char buffer[1] = {0}; 
    unsigned char dummy = 0x00; 
    unsigned char addressWidth = 0x01; // Variable to hold the address width
    unsigned char payload_size = 0x01; // Variable to hold the payload size
    unsigned char rfSetup = 0x00; // Variable to hold the RF setup value
    unsigned char configPRX = 0x0B; // Variable to hold the PRX mode config
    unsigned char configPowerDown = 0x09; // Variable to hold the power down config
    unsigned char rxAddress[] = {0x93, 0xBD, 0x6B}; // Variable to hold the RX address
    unsigned char clear_irqrx = 0x40; // Variable to hold the clear RX IRQ value for the status register
    unsigned char clear_ret = 0x10; // Variable to hold the clear retransmit value for the status register
    unsigned char clear = 0x01;
    unsigned char dummydata = 0xFF;

    commandNRF_SPI(FLUSH_RX_NRF); //send command to flush TX FIFO
    commandNRF_SPI(FLUSH_TX_NRF); //send command to flush TX FIFO
    //set control registers
    readwriteNRF_SPI(STATUS, &clear_ret, 1, WRITE_REG_NRF); 
    readwriteNRF_SPI(STATUS, &clear_irqrx, 1, WRITE_REG_NRF); 


    readwriteNRF_SPI(SETUP_AW, &addressWidth, 1, WRITE_REG_NRF); //set to 3 byte address width
    readwriteNRF_SPI(RX_ADDR_P0, rxAddress, 3, WRITE_REG_NRF); //set read address
    readwriteNRF_SPI(RX_PW_P0, &payload_size, 1, WRITE_REG_NRF); //set payload size
    
    readwriteNRF_SPI(RF_SETUP, &rfSetup, 1, WRITE_REG_NRF); //set RF Data Rate to 1Mbps, RF output power to -18dBm
    
    readwriteNRF_SPI(CONFIG_REG, &configPRX, 1, WRITE_REG_NRF); //set to PRX mode and set power on bit
    my_delay(200); 


    digitalWrite(2, HIGH); //enable chip to receive data by setting CE HIGH
    my_delay(1);
    my_delay(1);
    while(digitalRead(3)){ //wait for data to be received (IRQ pin is active low)
        my_delay(1);  //TODO add better delay function
    }          
    //digitalWrite(3, HIGH); //undo interrupt signal
    //delay(1000 * 2); //temporary delay to allow for data to be received by manual trigger

    readwriteNRF_SPI(0x00, buffer, 1, READ_PAYLOAD_NRF); //read data from RX FIFO
    digitalWrite(2, LOW); //switch chip to standby mode by setting CE pin low

    printf("Data received: %d\n", buffer[0]);
    readwriteNRF_SPI(CONFIG_REG, &configPowerDown, 1, WRITE_REG_NRF); //power down by writing to config register
}

/**
 * @brief  Delay function 
 * @param  milliseconds: specifies the delay time length, in 1 millisecond.
 * @retval None
 */
void my_delay(int milliseconds)
{
    long pause;
    clock_t now,then;

    pause = milliseconds*(CLOCKS_PER_SEC/1000);
    now = then = clock();
    while( (now-then) < pause )
        now = clock();
}





