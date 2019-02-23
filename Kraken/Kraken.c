#include <time.h>
#include <ctime>
#include <cmath>
#include "Kraken.h"

/*
 * This file gives all the variables needed to describe the Kraken
 * 
 * You can find:
 * - the different physical dimensions
 * - the type of controller and diverse stuctures used for control
 * 
 * it initializes and frees with two functions all what is needed.
 */
 

	

void init_Kraken(Kraken* myKraken)
{
	/*
	 *  Communication channels
	 */ 
	 
	//Set the GPIO from the rapsberry PI 
	if(wiringPiSetup() == -1)
	{
		printf("error : unable to set the GPI from the raspberry \n");
	}
	
	//Pin reset of the FPGA at the start.
	pinMode(resetPin, OUTPUT); 
	digitalWrite(resetPin, HIGH);
	delay(50);
	digitalWrite(resetPin, LOW);
	
	//Set the Can bus (Motor control)
	myKraken->can = new CAN(CAN_BR);
	myKraken->can->configure();
	
	//Set the SPI bus
	myKraken->spi;
	int fd = wiringPiSPISetup(channel, clockSpi);
	printf("SPI setup: %d \n", fd);
	
	
	/*
	 * Motion 
	 */
	 
	// Geometry
	myKraken->wheel_radius = 0.025; // m  
	myKraken->wheel_width = 0.22;   // m
	
	// Speed controller
	myKraken->motor_speed_left_wheel = 0.0;
	myKraken->motor_speed_right_wheel = 0.0;
	myKraken->odometer_speed_left_wheel = 0.0;
	myKraken->odometer_speed_right_wheel = 0.0;
	myKraken->angle_left_wheel = 0.0;
	myKraken->angle_right_wheel = 0.0;
	//myKraken->wheel_radiusspeed_left_enc;
	//myKraken->wheel_radiusspeed_left_enc;
	
	myKraken->theCtrlStruct = (CtrlStruct*) malloc (sizeof(struct CtrlStruct)); 
	myKraken->theCtrlStruct->theUserStruct = (UserStruct*) malloc (sizeof(struct UserStruct)); 
	myKraken->theCtrlStruct->theCtrlIn     = (CtrlIn*)     malloc (sizeof(struct CtrlIn)); 
	myKraken->theCtrlStruct->theCtrlOut    = (CtrlOut*)    malloc (sizeof(struct CtrlOut)); 
	init_speed_controller(myKraken->theCtrlStruct);
	
	//Position mapping
	myKraken->currentLocation = (PositionMapping*) malloc (sizeof(PositionMapping)); 
	initMapping(myKraken->currentLocation, myKraken->wheel_radius, myKraken->wheel_width = 10);
	reset_all(myKraken->currentLocation);
	
	//Trajectoy tracking
	myKraken->angularPositionPI = (PIcontroller*)  malloc (sizeof(PIcontroller));
	myKraken->linearPositionPI = (PIcontroller*)  malloc (sizeof(PIcontroller));	
	initPIController(myKraken->angularPositionPI, 1, 0, 0.001, 0, 2);
	initPIController(myKraken->linearPositionPI, 1, 0, 0.001, 0, 8);
	
	myKraken->tracker = (TrajectoryTracker*) malloc (sizeof(TrajectoryTracker)); 
	init_trajectoryTracker(myKraken->tracker, myKraken->currentLocation, myKraken->angularPositionPI, myKraken->linearPositionPI); // The point targeted by default is 0,0
	set_target(myKraken->tracker, 0, 0);
	//To be added a list that contains more than one trajectory
	
	//To be added ...
	//PIcontroller* target_angularPositionPI;
	//PIcontroller* target_linearPositionPI;
}

void free_Kraken(Kraken* myKraken)
{
	/*
	 * Motion 
	 */
	 
	//Speed controller
	free(myKraken->theCtrlStruct->theCtrlIn);
	free(myKraken->theCtrlStruct->theCtrlOut);
	free(myKraken->theCtrlStruct->theUserStruct);
	free(myKraken->theCtrlStruct);
	
	//position mapping
	free(myKraken->currentLocation);
	
	//trajectoy tracking
	free(myKraken->linearPositionPI);
	free(myKraken->angularPositionPI);
	free(myKraken->tracker);
	
	
}
