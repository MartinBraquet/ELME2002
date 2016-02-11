#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/SPI/Specific/SPI_CAN.hh"
#include "IO/COM/SPI/SPI.hh"
#include "Motor_control/CtrlStruct.h"
//#include "Direction_control/mappingPosition.h" // If using trajectoryTracker not usefull
//#include "Direction_control/direction_controller.h" // If using trajectoryTracker not usefull
#include "Direction_control/trajectoryTracker.h"
#include <time.h>
#include <ctime>
#include <cmath>

//Bus can
#define CAN_BR 500e3

//Bus SPI
#define channel 0
#define clockSpi 500000
//register data in the FPGA
#define LEFT_M_ENC 0x04;
#define RIGHT_M_ENC 0x05;

//RPI header
#define resetPin 25    //GPIO_O_PI[0]


/*
 * This file gives all the variables need to describe the Kraken
 * 
 * You can find:
 * - the different physical dimension
 * - the type of controller and diverse stucture used for control
 */
 
 typedef struct Kraken
{
	/*
	 *  Communication channels
	 */ 
	 CAN *can;
	 SPI *spi;

}Kraken;

void init_Kraken(Kraken* myKraken);
void free_Kraken(Kraken* myKraken);
