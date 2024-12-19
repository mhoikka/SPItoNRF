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

//Register addresses for NRF24L01+
unsigned char ENAA = 0x01;
unsigned char EN_RXADDR = 0x02;
unsigned char STATUS = 0x07;
unsigned char SETUP_AW = 0x03;
unsigned char RF_SETUP = 0x06;
unsigned char RX_ADDR_P0 = 0x0A;
unsigned char RX_PW_P0 = 0x11;
unsigned char TX_ADDR = 0x10;
unsigned char FIFO_STATUS = 0x17;

//Command addresses for NRF24L01+
unsigned char WRITE_PAYLOAD_NRF = 0xA0; //write TX FIFO command for NRF24L01+
unsigned char READ_PAYLOAD_NRF = 0x61;
unsigned char READ_RXWID_NRF = 0x60;
unsigned char FLUSH_TX_NRF = 0xE1;
unsigned char FLUSH_RX_NRF = 0xE2;


const unsigned char rxAddress[3] = {0x93, 0xBD, 0x6B}; // Variable to hold the RX address for NRF24L01+

const unsigned char ACK_PO1 = 0x01;
const unsigned char ADDRESS_WIDTH = 0x01; // Variable to hold the address width
const unsigned char PAYLOAD_SIZE = 0x20; // Variable to hold the payload size
const unsigned char RFSETUP = 0x00; // Variable to hold the RF setup value
const unsigned char CONFIGPRX = 0x0B; // Variable to hold the PRX mode config
const unsigned char CONFIGPOWERDOWN = 0x09; // Variable to hold the power down config

const unsigned char PIPE0 = 0x01; // Variable to hold the pipe 0 value
const unsigned char CLEAR_IRQRX = 0x40; // Variable to hold the clear RX IRQ value for the status register
const unsigned char CLEAR_IRQTX = 0x20; // Variable to hold the clear TX IRQ value for the status register
const unsigned char CLEAR_RET = 0x10; // Variable to hold the clear retransmit value for the status register

int main()
{
    int fd, result;
    unsigned char buffer[1] = {1}; //is this the right size?

    printf("Initializing\n");

    // Configure the interface.
    // CHANNEL insicates chip select,
    // 500000 indicates bus speed.
    fd = wiringPiSPISetup(CHANNEL, 500000); 

    int len = sizeof(buffer)/sizeof(buffer[0]);

    // Initialize WiringPi 
    wiringPiSetup();

    pinMode(15, OUTPUT); //set CE pin to output
    pinMode(3, INPUT); //set IRQ pin to input

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
 * @param command: Command to be sent to the NRF24L01+
 * @param readMode: Boolean value to determine if the data is being read or written
 */
void readwriteNRF_SPI(unsigned char reg_addr, unsigned char * buffer, int len, unsigned char command, int readMode){//TODO fix buffer argument to not be a pointer
    unsigned char new_buffer[len+1];
    unsigned char command_arr[1] = {command | reg_addr};
	int result;
	
	new_buffer[0] = command | reg_addr; 

    copy_Buffer(buffer, new_buffer, len, 1, command_arr);

    result = wiringPiSPIDataRW(CHANNEL, new_buffer, len+1);

    if (result == -1) {
        printf(stderr, "SPI communication failed\n");
        return; // Handle SPI error
    }
    if (readMode){
        copy_Buffer(&new_buffer[1], buffer, len, 0, command_arr);
    }
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
* @brief: receives a byte of data for testing purposes
*/
 //TODO make this much more functional
void receiveByteNRF(){

    unsigned char buffer[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,      
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF}; 
    unsigned char dummy = 0x00; 
    commandNRF_SPI(FLUSH_RX_NRF); //send command to flush RX FIFO
    commandNRF_SPI(FLUSH_TX_NRF); //send command to flush TX FIFO

    //set control registers
    readwriteNRF_SPI(STATUS, &CLEAR_RET, 1, WRITE_REG_NRF, 0); //reset interrupt bits
    readwriteNRF_SPI(STATUS, &CLEAR_IRQRX, 1, WRITE_REG_NRF, 0); 
    readwriteNRF_SPI(STATUS, &CLEAR_IRQTX, 1, WRITE_REG_NRF, 0); 

    readwriteNRF_SPI(SETUP_AW, &ADDRESS_WIDTH, 1, WRITE_REG_NRF, 0); //set to 3 byte address width
    readwriteNRF_SPI(RX_ADDR_P0, rxAddress, 3, WRITE_REG_NRF, 0); //set read address
    readwriteNRF_SPI(ENAA, &ACK_PO1, 1, WRITE_REG_NRF, 0); //enable auto-ack for pipe 0
    readwriteNRF_SPI(EN_RXADDR, &PIPE0, 1, WRITE_REG_NRF, 0); //set RX address to enable pipe 0
    readwriteNRF_SPI(RX_PW_P0, &PAYLOAD_SIZE, 1, WRITE_REG_NRF, 0); //set payload size 
    
    readwriteNRF_SPI(RF_SETUP, &RFSETUP, 1, WRITE_REG_NRF, 0); //set RF Data Rate to 1Mbps, RF output power to -18dBm
    
    readwriteNRF_SPI(CONFIG_REG, &CONFIGPRX, 1, WRITE_REG_NRF, 0); //set to PRX mode and set power on bit
    my_delay(2); 

    digitalWrite(15, HIGH); //enable chip to receive data by setting CE HIGH
    my_delay(1);
    my_delay(1);

    readwriteNRF_SPI(FIFO_STATUS, &dummy, 1, READ_REG_NRF, 0); //read FIFO status register
    readwriteNRF_SPI(STATUS, &dummy, 1, READ_REG_NRF, 0);

   while(1){
        readwriteNRF_SPI(STATUS, &dummy, 1, READ_REG_NRF, 1);
        while(!(dummy & (1 << 6))){                         //wait for data to be received 
            readwriteNRF_SPI(STATUS, &dummy, 1, READ_REG_NRF, 1);
        };        

        readwriteNRF_SPI(0x00, buffer, 32, READ_PAYLOAD_NRF, 1); //read data from RX FIFO

        printTempData(buffer, 32); //see what the temp data is

        //reset interrupt bits
        readwriteNRF_SPI(STATUS, &CLEAR_RET, 1, WRITE_REG_NRF, 0); 
        readwriteNRF_SPI(STATUS, &CLEAR_IRQRX, 1, WRITE_REG_NRF, 0); 
        readwriteNRF_SPI(STATUS, &CLEAR_IRQTX, 1, WRITE_REG_NRF, 0); 
    }

    digitalWrite(15, LOW); //switch chip to standby mode by setting CE pin low

    readwriteNRF_SPI(CONFIG_REG, &CONFIGPOWERDOWN, 1, WRITE_REG_NRF, 0); //power down by writing to config register

    readwriteNRF_SPI(STATUS, &CLEAR_RET, 1, WRITE_REG_NRF, 0); //reset interrupt bits
    readwriteNRF_SPI(STATUS, &CLEAR_IRQRX, 1, WRITE_REG_NRF, 0); 
    readwriteNRF_SPI(STATUS, &CLEAR_IRQTX, 1, WRITE_REG_NRF, 0); 
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
    printf("\n");
}

/**
 * @brief  print temp data
 * @param  buffer: pointer to buffer that will be printed
 * @param  len: length of buffer
 * @retval None
 */
void printTempData(unsigned char * buffer, int len){
    printf("\n");
    int temp = buffer[0] | buffer[1] << 8 | buffer[2] << 16 | buffer[3] << 24;
    printf("Ambient Temperature: %d C\n", temp);
    unsigned int pressure = buffer[4] | buffer[5] << 8 | buffer[6] << 16 | buffer[7] << 24;
    double pressure_kPa = pressure / 1000.0;
    printf("Ambient Pressure:    %.3lf kPa\n", pressure_kPa);
    unsigned int humidity = buffer[8]  | buffer[9] << 8 | buffer[10] << 16 | buffer[11] << 24;
    printf("Ambient Humidity:    %d%%\n", humidity);
    printf("\n");
}

/**
 * @brief  copy array with offset
 * @param  buffer: pointer to buffer that is being copied
 * @param  new_buffer: pointer to buffer that is being copied to
 * @param  len: length of buffer
 * @param  offset: offset before copying buffer into new_buffer
 * @param  command: unsigned char command to be copied into index 0 of new_buffer
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





