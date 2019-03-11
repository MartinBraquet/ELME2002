#include "SPI_DE0.hh"


SPI_DE0::SPI_DE0(unsigned int CS, unsigned int baud): SPI(CS,baud)
{

}

SPI_DE0::~SPI_DE0()
{
}




void SPI_DE0::writeSPI(unsigned int ad, int data)
{
    	unsigned char bytes[8] = {0};

	tobytes(4, 0x80000000 | ad, bytes);
	tobytes(4, data, &bytes[4]);
	
	wiringPiSPIDataRW(cs, bytes, 8);
}
	
	
unsigned int SPI_DE0::readSPI(unsigned int ad)
{
	unsigned char bytes[8] = {0};
    	tobytes(4, ad, bytes);
    	wiringPiSPIDataRW(cs, bytes, 8);
	return frombytes(4, &bytes[4]);
}


void SPI_DE0::write(unsigned int ad, unsigned int data)
{
	writeSPI(0x10, ad);
	writeSPI(0x20, data);	

}

unsigned int SPI_DE0::read(unsigned int ad)
{
	writeSPI(0x10, ad);
	return readSPI(0x20);	

}