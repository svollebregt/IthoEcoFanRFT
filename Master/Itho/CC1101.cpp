/*
 * Author: Klusjesman, modified bij supersjimmie for Arduino/ESP8266
 */

#include "CC1101.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

// default constructor
CC1101::CC1101()
{
	int x = 0;
     	//printf ("init SPI bus... ");
     	if ((x = wiringPiSPISetup (0, 8000000)) < 0)  //4MHz SPI speed
     	{
        	printf ("ERROR: wiringPiSPISetup failed!\r\n");
     	}
     	else{
          	//printf ("wiringSPI is up\r\n");
        }
//	SPI.begin();
//#ifdef ESP8266
//	pinMode(SS, OUTPUT);
//#endif
} //CC1101

// default destructor
CC1101::~CC1101()
{
} //~CC1101

/***********************/
// SPI helper functions select() and deselect()
inline void CC1101::select(void) {
	digitalWrite(SS, LOW);
}

inline void CC1101::deselect(void) {
	digitalWrite(SS, HIGH);
}

void CC1101::spi_waitMiso()
{
    while(digitalRead(MISO) == HIGH) yield();
}

void CC1101::init()
{
	reset();
}

void CC1101::reset()
{
	deselect();
	delayMicroseconds(5);
	select();
	delayMicroseconds(10);
	deselect();
	delayMicroseconds(45);
	select();

	writeCommand(CC1101_SRES);
	delay(10);
	spi_waitMiso();
	deselect();
}

void CC1101::writeCommand(uint8_t command) //not sure why this was unint8_t as the output seems to be never used?
{
	//uint8_t result;
	
	uint8_t tbuf[1] = {0};
	tbuf[0] = command;
	wiringPiSPIDataRW (0, tbuf, 1) ;
	
	//select();
	//spi_waitMiso();
	//result = SPI.transfer(command);
	//deselect();
	
	//return result;
}

void CC1101::writeRegister(uint8_t address, uint8_t data) 
{
	uint8_t tbuf[2] = {0};
	tbuf[0] = address;
	tbuf[1] = data;
	uint8_t len = 2;
	wiringPiSPIDataRW (0, tbuf, len) ;
	
	//select();
	//spi_waitMiso();
	//SPI.transfer(address);
	//SPI.transfer(data);
	//deselect();
}

uint8_t CC1101::readRegister(uint8_t address)
{
	uint8_t val;
	uint8_t rbuf[2] = {0};
	rbuf[0] = spi_instr;
	uint8_t len = 2;
	wiringPiSPIDataRW (0, rbuf, len) ;
	val = rbuf[1];
	//select();
	//spi_waitMiso();
	//SPI.transfer(address);
	//val = SPI.transfer(0);
	//deselect();
  
	return val;
}

uint8_t CC1101::readRegisterMedian3(uint8_t address)
{
  uint8_t val, val1, val2, val3;

  select();
  spi_waitMiso();
  SPI.transfer(address);
  val1 = SPI.transfer(0);
  SPI.transfer(address);
  val2 = SPI.transfer(0);
  SPI.transfer(address);
  val3 = SPI.transfer(0);
  deselect();
  // reverse sort (largest in val1) because this is te expected order for TX_BUFFER
  if (val3 > val2) {val = val3; val3 = val2; val2 = val; } //Swap(val3,val2)
  if (val2 > val1) {val = val2; val2 = val1, val1 = val; } //Swap(val2,val1)
  if (val3 > val2) {val = val3; val3 = val2, val2 = val; } //Swap(val3,val2)
  
  return val2;
}

/* Known SPI/26MHz synchronization bug (see CC1101 errata)
This issue affects the following registers: SPI status byte (fields STATE and FIFO_BYTES_AVAILABLE), 
FREQEST or RSSI while the receiver is active, MARCSTATE at any time other than an IDLE radio state, 
RXBYTES when receiving or TXBYTES when transmitting, and WORTIME1/WORTIME0 at any time.*/
//uint8_t CC1101::readRegisterWithSyncProblem(uint8_t address, uint8_t registerType)
uint8_t /* ICACHE_RAM_ATTR */ CC1101::readRegisterWithSyncProblem(uint8_t address, uint8_t registerType)
{
	uint8_t value1, value2;	
	
	value1 = readRegister(address | registerType);
	
	//if two consecutive reads gives us the same result then we know we are ok
	do 
	{
		value2 = value1;
		value1 = readRegister(address | registerType);
	} 
	while (value1 != value2);
	
	return value1;
}

//registerType = CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER
uint8_t CC1101::readRegister(uint8_t address, uint8_t registerType)
{
	switch (address)
	{
		case CC1101_FREQEST:
		case CC1101_MARCSTATE:
		case CC1101_RXBYTES:
		case CC1101_TXBYTES:
		case CC1101_WORTIME1:
		case CC1101_WORTIME0:	
			return readRegisterWithSyncProblem(address, registerType);	
			
		default:
			return readRegister(address | registerType);
	}
}

void CC1101::writeBurstRegister(uint8_t address, uint8_t* data, uint8_t length)
{
	uint8_t tbuf[length + 1];
	tbuf[0] = address | WRITE_BURST;
	for (uint8_t i=0; i<length ;i++ )
	{
		tbuf[i+1] = data[i];
	}
	wiringPiSPIDataRW (0, tbuf, length + 1) ;
	
	//uint8_t i;

	//select();
	//spi_waitMiso();
	//SPI.transfer(address | CC1101_WRITE_BURST);
	//for (i = 0; i < length; i++) {
	//	SPI.transfer(data[i]);
	//}
	//deselect();
}

void CC1101::readBurstRegister(uint8_t* buffer, uint8_t address, uint8_t length)
{
	uint8_t rbuf[length + 1];
	rbuf[0] = address | C1101_READ_BURST;
	wiringPiSPIDataRW (0, rbuf, length + 1) ;
	for (uint8_t i=0; i<length ;i++ )
	{
		buffer[i] = rbuf[i+1];
	}
	
	
	//nt8_t i;
	
	//select();
	//spi_waitMiso();
	//SPI.transfer(address | CC1101_READ_BURST);
	
	//for (i = 0; i < length; i++) {
	//	buffer[i] = SPI.transfer(0x00);
	//}
	
	//deselect();
}

//wait for fixed length in rx fifo
uint8_t CC1101::receiveData(CC1101Packet* packet, uint8_t length)
{
	uint8_t rxBytes = readRegisterWithSyncProblem(CC1101_RXBYTES, CC1101_STATUS_REGISTER);
	rxBytes = rxBytes & CC1101_BITS_RX_BYTES_IN_FIFO;
	
	//check for rx fifo overflow
	if ((readRegisterWithSyncProblem(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & CC1101_BITS_MARCSTATE) == CC1101_MARCSTATE_RXFIFO_OVERFLOW)
	{
		writeCommand(CC1101_SIDLE);	//idle
		writeCommand(CC1101_SFRX); //flush RX buffer
		writeCommand(CC1101_SRX); //switch to RX state	
	}
	else if (rxBytes == length)
	{
		readBurstRegister(packet->data, CC1101_RXFIFO, rxBytes);

		//continue RX
		writeCommand(CC1101_SIDLE);	//idle		
		writeCommand(CC1101_SFRX); //flush RX buffer
		writeCommand(CC1101_SRX); //switch to RX state	
		
		packet->length = rxBytes;				
	}
	else
	{
		//empty fifo
		packet->length = 0;
	}

	return packet->length;
}

//This function is able to send packets bigger then the FIFO size.
void CC1101::sendData(CC1101Packet *packet)
{
	uint8_t index = 0;
	uint8_t txStatus, MarcState;
	uint8_t length;
	
	writeCommand(CC1101_SIDLE);		//idle

	txStatus = readRegisterWithSyncProblem(CC1101_TXBYTES, CC1101_STATUS_REGISTER);
		
	//clear TX fifo if needed
	if (txStatus & CC1101_BITS_TX_FIFO_UNDERFLOW)
	{
		writeCommand(CC1101_SIDLE);	//idle
		writeCommand(CC1101_SFTX);	//flush TX buffer
	}	
	
	writeCommand(CC1101_SIDLE);		//idle	
	
	//determine how many bytes to send
	length = (packet->length <= CC1101_DATA_LEN ? packet->length : CC1101_DATA_LEN);
	
	writeBurstRegister(CC1101_TXFIFO, packet->data, length);

	writeCommand(CC1101_SIDLE);
	//start sending packet
	writeCommand(CC1101_STX);		

	//continue sending when packet is bigger than 64 bytes
	if (packet->length > CC1101_DATA_LEN)
	{
		index += length;
		
		//loop until all bytes are transmitted
		while (index < packet->length)
		{
			//check if there is free space in the fifo
			while ((txStatus = (readRegisterMedian3(CC1101_TXBYTES | CC1101_STATUS_REGISTER) & CC1101_BITS_RX_BYTES_IN_FIFO)) > (CC1101_DATA_LEN - 2));
			
			//calculate how many bytes we can send
			length = (CC1101_DATA_LEN - txStatus);
			length = ((packet->length - index) < length ? (packet->length - index) : length);
			
			//send some more bytes
			for (int i=0; i<length; i++)
				writeRegister(CC1101_TXFIFO, packet->data[index+i]);
			
			index += length;			
		}
	}

	//wait until transmission is finished (TXOFF_MODE is expected to be set to 0/IDLE or TXFIFO_UNDERFLOW)
	do
	{
		MarcState = (readRegisterWithSyncProblem(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & CC1101_BITS_MARCSTATE);
//		if (MarcState == CC1101_MARCSTATE_TXFIFO_UNDERFLOW) Serial.print(F("TXFIFO_UNDERFLOW occured in sendData() \n"));
	}
  	while((MarcState != CC1101_MARCSTATE_IDLE) && (MarcState != CC1101_MARCSTATE_TXFIFO_UNDERFLOW));
}
