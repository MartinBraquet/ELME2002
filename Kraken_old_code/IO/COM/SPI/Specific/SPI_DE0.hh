#ifndef _SPI_DE0_HH_
#define _SPI_DE0_HH_

#include "../SPI.hh"

// Writing addresses
#define A_LED 0x51

// Reading addresses
#define A_LW 0x1

class SPI_DE0: public SPI
{
	public:
		SPI_DE0(unsigned int CS, unsigned int baud);
		~SPI_DE0();

	
		void writeSPI(unsigned int ad, int data);
		unsigned int readSPI(unsigned int ad);

		void write(unsigned int ad, unsigned int value);
		unsigned int read(unsigned int ad);

};


#endif

