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
	 
	
	/*
	 * Motion 
	 */
	 
	//Variables used to define the physical measure
	////////////////////////////////////////////////
	double wheel_radius;
	double wheel_width; 
	
	//Variables used to store computed informations
	////////////////////////////////////////////////
	double angle_left_wheel;
	double angle_right_wheel;
	double motor_speed_left_wheel;
	double motor_speed_right_wheel;
	double odometer_speed_right_wheel;
	double odometer_speed_left_wheel;
	
	// General variable to keep track of the current location
	PositionMapping* currentLocation;
	// General variable to keep track of location to reach
	PIcontroller* angularPositionPI;
	PIcontroller* linearPositionPI;
	TrajectoryTracker* tracker;
	// Structure needed for the wheel motor controle
	CtrlStruct* theCtrlStruct;
	
	
	
	/*
	 * Here below you can add other features to the kraken for others field than the motion control
	 */
	 
	//....
}Kraken;

void init_Kraken(Kraken* myKraken);
void free_Kraken(Kraken* myKraken);
