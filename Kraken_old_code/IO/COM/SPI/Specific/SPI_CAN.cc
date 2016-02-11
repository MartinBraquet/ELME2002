#include "SPI_CAN.hh"
#include <cstdio>

// def readReg(address, len):
//     bytes = [0b11, address] + [0x00,]*len
//     resp = s.xfer2(bytes)
//     return resp[2:len+2]

// def readRXBuf(buf_nb, len):
//     bytes = [0b10010010 | buf_nb << 2] + [0x00,]*len
//     resp = s.xfer2(bytes)
//     return resp[1:len+2]

// def loadTXBuf(buf_nb, data):
//     bytes=[0b01000001 | buf_nb << 1] + data
//     resp = s.xfer2(bytes)

// def writeReg(address, data):
//     bytes=[0b10, address] + data
//     #print("sending :")
//     #prthexlist(bytes) 
//     resp = s.xfer2(bytes)

// def getBitsField(reg_ad, pos, len):
//     reg = readReg(reg_ad, 1)
//     return (reg[0] >> pos)%(2**len)

// def bitModify(reg_ad, mask, data):
//     bytes=[0b101, reg_ad, mask, data]
//     resp = s.xfer2(bytes)

// def readStatus():
//     bytes = [0b10100000, 0x0] 
//     resp = s.xfer2(bytes)
//     return resp[1]


SPI_CAN::SPI_CAN(unsigned int CS, unsigned int baud): SPI(CS,baud)
{
	
}

SPI_CAN::~SPI_CAN()
{
	
}

void SPI_CAN::writeReg(uint8_t ad, uint8_t len, uint8_t *buffer)
{
	unsigned char *bytes = new unsigned char(len+2);
	bytes[0] = 0b10;
	bytes[1] = ad;

	for(int i=0; i<len; ++i)
		bytes[i+2] = buffer[i];

	wiringPiSPIDataRW(cs, bytes, len+2);
}

void SPI_CAN::readReg(uint8_t ad, uint8_t len, uint8_t *buffer)
{
	unsigned char *bytes = new unsigned char(len+2);
	bytes[0] = 0b11;
	bytes[1] = ad;

	for(int i=0; i<len; ++i)
		bytes[i+2] = 0;

	wiringPiSPIDataRW(cs, bytes, len+2);

	for(int i=0; i<len; ++i)
		buffer[i] = bytes[i+2];

}

void SPI_CAN::readRXBuf(uint8_t buf_nb, uint8_t len, uint8_t *buffer)
{
	unsigned char *bytes = new unsigned char(len+2);
	bytes[0] = 0b10010010 | buf_nb << 2;

	for(int i=0; i<len; ++i)
		bytes[i+1] = buffer[i];

	wiringPiSPIDataRW(cs, bytes, len+1);

	for(int i=0; i<len; ++i)
		buffer[i] = bytes[i+1];
}

void SPI_CAN::loadTXBuf(uint8_t buf_nb, uint8_t len, uint8_t *buffer)
{
	unsigned char *bytes = new unsigned char(len+1);
	bytes[0] = 0b01000001 | buf_nb << 1;

	for(int i=0; i<len; ++i){
		bytes[i+1] = buffer[i];
	}

	wiringPiSPIDataRW(cs, bytes, len+1);
}

uint8_t SPI_CAN::getBitsField(uint8_t ad, uint8_t pos, uint8_t len)
{
	unsigned char buffer;

	readReg(ad, 1, &buffer);
	return (buffer >> pos) % (0b1 << (len));
}

void SPI_CAN::bitModify(uint8_t ad, uint8_t mask, uint8_t value)
{
	unsigned char bytes[4];
	bytes[0] = 0b101; 
	bytes[1] = ad;
	bytes[2] = mask;
	bytes[3] = value;

	wiringPiSPIDataRW(cs, bytes, 4);
}

uint8_t SPI_CAN::readStatus()
{
	unsigned char bytes[2];
	bytes[0] = 0b10100000;
	bytes[1] = 0x0;

	wiringPiSPIDataRW(cs, bytes, 2);

	return bytes[1];
}