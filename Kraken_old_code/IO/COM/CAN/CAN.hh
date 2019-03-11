#ifndef _CAN_HH_
#define _CAN_HH_

#include <stdint.h>
#include "../SPI/Specific/SPI_CAN.hh"
#include <cmath>

const int CAN_BD = 500e3;

const int FOSC = 16000000;

const int TXB_CTRL[3] = {0x30, 0x40, 0x50 }; //Transmit buffer ctrl

const int TXB_SIDH[3] = { 0x31, 0x41, 0x51 };
const int TXB_SIDL[3] = { 0x32, 0x42, 0x52 };
const int TXB_DLC[3]  = { 0x35, 0x45, 0x55 };
const int TXB_DATA[3] = { 0x36, 0x46, 0x56 };

const int RXB_CTRL[3] = { 0x60, 0x70 };
const int BFPCTRL  = 0x0C;

const int RXB_SIDH[3] = { 0x61, 0x71 };
const int RXB_SIDL[3] = { 0x62, 0x72 };
const int RXB_DLC[3]  = { 0x65, 0x75 };
const int RXB_DATA[3] = { 0x66, 0x76 };

const int CANCTRL = 0xF;
const int CANSTAT = 0xE;

const int CANINTE = 0x2B;
const int CANINTF = 0x2C;

const int CNF1 = 0x2A;
const int CNF2 = 0x29;
const int CNF3 = 0x28;

const int EFLG = 0x2D;

#define SWR 0
#define SWL 1

#define CAN_MOT 0x408
#define CAN_TOW 0x400

typedef struct CANMessage{
	uint16_t sid;
	uint8_t len;
	uint8_t rtr;
	uint8_t priority;
	uint8_t data[4]; 
}CANMessage;


class CAN{
	public:
		CAN(unsigned int baudrate);
		~CAN();

		void configure();
		void check_receive();
		int readMessage(CANMessage *msg, uint8_t buf_nb);
		int sendMessage(CANMessage *msg, uint8_t buf_nb);
		void decipher_msg(CANMessage *msg);

		inline int get_usw(int side) {return sw[side];}

		void push_TowDC(int dc);
		void push_PropDC(int dcG, int dcD);
		void ctrl_led(int state);
		void ctrl_motor(int state);

	private:
		SPI_CAN *spi2can;
		uint32_t baudrate;

		int sw[2];

};

#endif
