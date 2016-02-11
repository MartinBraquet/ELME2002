#ifndef _SPI_CAN_HH_
#define _SPI_CAN_HH_

#include <stdint.h>
#include "../SPI.hh"

class SPI_CAN: public SPI
{
	public:
		SPI_CAN(unsigned int CS, unsigned int baud);
		~SPI_CAN();
		
		void writeReg(uint8_t ad, uint8_t len, uint8_t *buffer);
		void readReg(uint8_t ad, uint8_t len, uint8_t *buffer);
		void readRXBuf(uint8_t buf_nb, uint8_t len, uint8_t *buffer);
		void loadTXBuf(uint8_t buf_nb, uint8_t len, uint8_t *buffer);
		uint8_t getBitsField(uint8_t ad, uint8_t pos, uint8_t len);
		void bitModify(uint8_t ad, uint8_t mask, uint8_t value);
		uint8_t readStatus();



	private:


};



#endif