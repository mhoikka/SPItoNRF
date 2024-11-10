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
unsigned char EN_RXADDR = 0x02;
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
unsigned char FIFO_STATUS = 0x17;

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

    pinMode(15, OUTPUT); //set CE pin to output
    pinMode(3, INPUT); //set IRQ pin to input
    //pullUpDnControl(3, PUD_UP); //enable pull-up resistor on IRQ pin
    pullUpDnControl(15, PUD_DOWN); //enable pull-down resistor on CE pin
    digitalWrite(15, LOW); 

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
    //memcpy(&new_buffer[1], buffer, len * sizeof(unsigned char));
    //command_copy_Buffer(buffer, new_buffer, len, command);
    copy_Buffer(buffer, new_buffer, len, 1, &command);

    printf("%d ", new_buffer[1]);
    result = wiringPiSPIDataRW(CHANNEL, new_buffer, len+1);
    printf("%d\n", new_buffer[1]);
    if (result == -1) {
        printf(stderr, "SPI communication failed\n");
        return; // Handle SPI error
    }
    //memcpy(buffer, &new_buffer[1], len * sizeof(unsigned char));
    copy_Buffer(new_buffer[1], buffer, len, 0, &command);
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
    /*unsigned char buffer[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,      
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF}; */
    unsigned char buffer[1] = {0xFF};
    unsigned char dummy = 0x00; 
    unsigned char no_ack = 0x00;
    unsigned char addressWidth = 0x01; // Variable to hold the address width
    unsigned char payload_size = 0x01; // Variable to hold the payload size
    unsigned char rfSetup = 0x00; // Variable to hold the RF setup value
    unsigned char configPRX = 0x0B; // Variable to hold the PRX mode config
    unsigned char configPowerDown = 0x09; // Variable to hold the power down config
    unsigned char rxAddress[3] = {0x93, 0xBD, 0x6B}; // Variable to hold the RX address
    unsigned char pipe0 = 0x01; // Variable to hold the pipe 0 value
    unsigned char clear_irqrx = 0x40; // Variable to hold the clear RX IRQ value for the status register
    unsigned char clear_irqtx = 0x20; // Variable to hold the clear TX IRQ value for the status register
    unsigned char clear_ret = 0x10; // Variable to hold the clear retransmit value for the status register
    unsigned char clear = 0x01;
    unsigned char dummydata = 0xFF;

    commandNRF_SPI(FLUSH_RX_NRF); //send command to flush RX FIFO
    commandNRF_SPI(FLUSH_TX_NRF); //send command to flush TX FIFO
    //set control registers
    readwriteNRF_SPI(STATUS, &clear_ret, 1, WRITE_REG_NRF); //reset interrupt bits
    readwriteNRF_SPI(STATUS, &clear_irqrx, 1, WRITE_REG_NRF); 
    readwriteNRF_SPI(STATUS, &clear_irqtx, 1, WRITE_REG_NRF); 


    readwriteNRF_SPI(SETUP_AW, &addressWidth, 1, WRITE_REG_NRF); //set to 3 byte address width
    readwriteNRF_SPI(RX_ADDR_P0, rxAddress, 3, WRITE_REG_NRF); //set read address
    //readwriteNRF_SPI(ENAA, &no_ack, 1, WRITE_REG_NRF); //disable auto-ack
    readwriteNRF_SPI(EN_RXADDR, &pipe0, 1, WRITE_REG_NRF); //set RX address to enable pipe 0
    //readwriteNRF_SPI(RX_PW_P0, &payload_size, 1, WRITE_REG_NRF); //set payload size
    
    readwriteNRF_SPI(RF_SETUP, &rfSetup, 1, WRITE_REG_NRF); //set RF Data Rate to 1Mbps, RF output power to -18dBm
    
    readwriteNRF_SPI(CONFIG_REG, &configPRX, 1, WRITE_REG_NRF); //set to PRX mode and set power on bit
    my_delay(2); 


    digitalWrite(15, HIGH); //enable chip to receive data by setting CE HIGH
    my_delay(1);
    my_delay(1);

    readwriteNRF_SPI(FIFO_STATUS, &dummy, 1, READ_REG_NRF); //read FIFO status register
    readwriteNRF_SPI(STATUS, &dummy, 1, READ_REG_NRF);

    readwriteNRF_SPI(0x00, buffer, 1, READ_PAYLOAD_NRF); //read data from RX FIFO
    printf("Past data received: %d\n", buffer[0]);

    while(digitalRead(3)){ //wait for data to be received (IRQ pin is active low)
        my_delay(1);  //TODO add better delay function
    }          
    //digitalWrite(3, HIGH); //undo interrupt signal
    //delay(1000 * 2); //temporary delay to allow for data to be received by manual trigger

    readwriteNRF_SPI(0x00, buffer, 1, READ_PAYLOAD_NRF); //read data from RX FIFO
    digitalWrite(15, LOW); //switch chip to standby mode by setting CE pin low

    printf("Data received: %d\n", buffer[0]);
    readwriteNRF_SPI(CONFIG_REG, &configPowerDown, 1, WRITE_REG_NRF); //power down by writing to config register

    readwriteNRF_SPI(STATUS, &clear_ret, 1, WRITE_REG_NRF); //reset interrupt bits
    readwriteNRF_SPI(STATUS, &clear_irqrx, 1, WRITE_REG_NRF); 
    readwriteNRF_SPI(STATUS, &clear_irqtx, 1, WRITE_REG_NRF); 
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

/**
 * @brief  print array 
 * @param  buffer: pointer to buffer that will be printed
 * @param  len: length of buffer
 * @retval None
 */
void printBuffer(unsigned char * buffer, int len){
    printf("\n");
    for(int i = 0; i < len; i++){
        printf("%d ", buffer[i]);
    }
}

/**
 * @brief  copy array with offset
 * @param  buffer: pointer to buffer that will be printed
 * @param  new_buffer: pointer to buffer that will be printed
 * @param  len: length of buffer
 * @param  offset: offset to start copying from
 * @retval None
 */
void copy_Buffer(unsigned char * buffer, unsigned char * new_buffer, int len, int offset, unsigned char * command){
    for(int i = 0; i < offset; i++){
        new_buffer[i] = command[i];
    }
    for(int i = 0; i < len; i++){
        new_buffer[i+offset] = buffer[i];
    }
    
}





