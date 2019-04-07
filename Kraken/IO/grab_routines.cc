#include "grab_routines.h"

//void grabAtoms1(unsigned char *buffer, SPI *spi) {
//	unsigned char pneumatics_state;
//	wiringPiSPIDataRW(channel, buffer, 5);
//	pneumatics_state = (unsigned char) spi->frombytes(5,buffer);
//
//	//dynamixel bottom to middle
//	dynamixelSetWheelMode(DYNAMIXEL_ELEVATOR_ID);
//	dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID,0x3ff+0x400);
//	usleep(2.15*1000000);
//	dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID,0);
//	usleep(100000);
//
//	// turn on venteuse
//	buffer[0] |= VOID_PUMP_1_MASK;
//	wiringPiSPIDataRW(channel)
//
//	//piston 1 and 2 +
//	buffer[0] |= PISTON_1_MASK;
//	buffer[0] |= PISTON_2_MASK;
//
//	
//
//}