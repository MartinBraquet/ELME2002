#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/SPI/Specific/SPI_CAN.hh"
#include "IO/COM/SPI/SPI.hh"
#include "Motor_control/CtrlStruct.h"
#include "Direction_control/direction_controller.h"
//#include "Direction_control/mappingPosition.h" // If using trajectoryTracker not usefull
#include "Direction_control/trajectoryTracker.h"
#include <time.h>
#include <ctime>
#include <cmath>



#define CAN_BR 125e3

//Define all the pins from the raspberry PI connected to the FPGA 
#define resetPin 25    //GPIO_O_PI[0]
#define laserSignal 24 //GPIO_O_PI[1]
#define laserSync 23   //GPIO_O_PI[3]
//#define laserCodeA 22  //GPIO_O_PI[5] not use 
//#define laserCodeB 21  //GPIO_O_PI[7] not use

#define channel 0
#define clockSpi 500000

#define set 1
#define setWheel 1
#define setTurret 1
#define print 0
#define turnMotorsON_OFF 1


// Angle of the turret in degree
double compute_angle_turret(int count){
	double angle = count * 360.0 / 7200.0;
	int angle_rounded = ceil(angle);
	double digit = (angle) - ceil(angle);
	if(count >= 0)
		return angle_rounded % 360 + digit - 180;
	else
		return angle_rounded % 360 + digit + 180;
}

// Rotation speed of the turret in rad/s: compute each 65536 cycles and the CLK is at 50 MHz
double compute_speed_turret(int speed){
	return (speed * 2 * M_PI / 7200.0 /  65536.0  * 50e6);
}
	

int main()
{
	printf("hello world\n");
	printf("##############################################################################################################\n");
    	printf("\t\t\tWelcome to the Minibot project of the ELME2002 class :)");
    	printf("##############################################################################################################\n");
    	printf("\t\t I'm Mister GrayCode, please take care of me !\n");
    	printf("\t\t Please do not interchange the chips on my tower/motor PWM boards !\n");
    	printf("\t\t Try to respect the C-file interface when programming me because\n \t\t it will be the same in the robotic project (Q2) !\n");

	CAN *can;
	can = new CAN(CAN_BR);
	can->configure();
	can->ctrl_led(0);
	can->ctrl_motor(turnMotorsON_OFF);
	SPI *spi;
	SPI_CAN *spi_can;
	int t = 1;
	

	


	//SETTING THE SPI INTERFACE
	////////////////////////////////////////////////////////////////////////////////
	
	int fd = wiringPiSPISetup(channel, clockSpi);
	printf("SPI setup: %d \n",fd);
	unsigned char buffer[100];

	//SETTING THE GPIO
	////////////////////////////////////////////////////////////////////////////////

	//Set the GPIO from the rapsberry PI 
	if(wiringPiSetup() == -1)
	{
		printf("error : unable to set the GPI from the raspberry \n");
	}

	//Pin reset
	pinMode(resetPin,OUTPUT); 

	//Pins connected to the laser
	pinMode(laserSignal,INPUT);	
	pinMode(laserSync,INPUT);

	//RESETING FPGA AND CHOOSING TURRET ANGLE REFERENCE
	////////////////////////////////////////////////////////////////////////////////
	
	digitalWrite(resetPin,HIGH);
	delay(50);
	digitalWrite(resetPin,LOW);

	can->push_TowDC(0);
	//To set the initial angle
	printf("Please set the tower to the desired angle of reference... \n");	
	delay(5000);

	//reset the FPGA at the begining
	printf("Restarting... \n");
	digitalWrite(resetPin,HIGH);
	delay(50);
	digitalWrite(resetPin,LOW);

	//VARIABLES
	////////////////////////////////////////////////////////////////////////////////
	//Function properties
	int backHome = 1;
	int onTarget = 1;

	//set robot propreties
	double WHEELS_DISTANCE =  0.226; //m
	double WHEEL_RADIUS =  0.0292; //m
	double angleRobot = 0; 


	double omref_rotation = 0.0;
	double omref_forward = 0.0;
	double omref = omref_rotation+omref_forward;
	double omega_ref[2] = {omref,omref};
	double angle_l_before, angle_r_before, angle_l_after, angle_r_after;
	
	double turret_speed = 0.0;
	double count_L = 0.0; double count_R  =0.0;
	double angle_turret = 0, angle_left_wheel = 0, angle_right_wheel = 0;
	double speed_turret = 0, speed_left_wheel = 0, speed_right_wheel = 0;
	double old_angle_turret = 0;
	double angleIn = 0.0;
	double angleOut = 0.0;
	double deltaAngle = 0.0;
	int obstacle_detected = 0; 
	int obstacle_was_detected = 1; 
	int obstacle_detected_onThisTurn = 0;
	int not_found_max = 10;
	int target_not_found = not_found_max+1;
	int close_to_target = 0;
	
	
	//CONTOLLER
	////////////////////////////////////////////////////////////////////////////////
	//Speed controller
	CtrlStruct* theCtrlStruct = (CtrlStruct*) malloc (sizeof(struct CtrlStruct)); 
	theCtrlStruct->theUserStruct = (UserStruct*) malloc (sizeof(struct UserStruct)); 
	theCtrlStruct->theCtrlIn     = (CtrlIn*)     malloc (sizeof(struct CtrlIn)); 
	theCtrlStruct->theCtrlOut    = (CtrlOut*)    malloc (sizeof(struct CtrlOut)); 
	init_speed_controller(theCtrlStruct);


	//Going on the beacon
	//controller for the straight direction
	PIcontroller* target_angularPositionPI = (PIcontroller*) malloc (sizeof(PIcontroller)); 
	double kp = 12;
	double ki = 0;
	initPIController(target_angularPositionPI,kp,ki);
	double distanceRef = 25;
	double distanceMes = 0.0;

	//proportionnal controller for the rotationnal speed
	PIcontroller* target_linearPositionPI = (PIcontroller*) malloc (sizeof(PIcontroller)); 
	kp = 25;
	ki = 0.0;
	initPIController(target_linearPositionPI,kp,ki);
	double angleRef  = 0.0;
	double angleMes  = 0.0;
	
	
	//Going back home
	PIcontroller* home_angularPositionPI = (PIcontroller*) malloc (sizeof(PIcontroller)); 
	kp = 12;
	ki = 0;
	initPIController(home_angularPositionPI,kp,ki);

	PIcontroller* home_linearPositionPI = (PIcontroller*) malloc (sizeof(PIcontroller)); 
	kp = 20; //rad/s
	ki = 0;
	initPIController(home_linearPositionPI,kp,ki);	
	
	double dt_PI = 0.001;

	//POSTION MAPPING
	/////////////////////////////////////////////////////////////////////////////////
	PositionMapping* positionMap = (PositionMapping*) malloc (sizeof(PositionMapping)); 
	initMapping(positionMap,WHEEL_RADIUS,WHEELS_DISTANCE);
	

	//TRAJECTORY TRACKING 

	////////////////////////////////////////////////////////////////////////////////
	TrajectoryTracker* tracker =(TrajectoryTracker*) malloc (sizeof(TrajectoryTracker)); 
	init_trajectoryTracker(tracker,positionMap); // The point targeted by default is 0,0
	set_target(tracker,0,0);

	can->push_PropDC(10,10);
	can->push_TowDC(0);
	delay(5000);
	reset_all(positionMap);
	print_mapping(positionMap);
	print_tracker(tracker);
	
	while(true){
		
		
		delay(1); // delay of the speed controller, do not change it ! 

		//GETTING DATA FROM SPI
		///////////////////////////////////////////////////////////////////////////////////////
		
		// Data from turret position
		buffer[0] = 0x00;
		
		//printf("Send data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
	   	wiringPiSPIDataRW(channel, buffer, 5);
		//printf("Received data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
		old_angle_turret = angle_turret;
		angle_turret = compute_angle_turret(spi->frombytes(5, buffer));
		if (print)
	        {
			printf("angle turret: %f \n", angle_turret);
		}

		// Data from left wheel position
		buffer[0] = 0x01;
		
		//printf("Send data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
	   	wiringPiSPIDataRW(channel, buffer, 5);
		//printf("Received data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
		count_L = spi->frombytes(5, buffer);
		angle_left_wheel = compute_angle_wheel_motor(count_L);
		if (print) {
			printf("angle left wheel: %f \n", angle_left_wheel);
		}


		// Data from right wheel position
		buffer[0] = 0x02;
		//printf("Send data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
	   	wiringPiSPIDataRW(channel, buffer, 5);
		//printf("Received data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
		count_R = spi->frombytes(5, buffer);
		angle_right_wheel = compute_angle_wheel_motor(count_R);
		if (print) {
			printf("angle right wheel: %f \n", angle_right_wheel);
		}
		
		// Data from lasersync
		buffer[0] = 0x0F;
		//printf("Send data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
	   	wiringPiSPIDataRW(channel, buffer, 5);
		//printf("Lasersync : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
		
		// Data from turret speed
		buffer[0] = 0x03;
		
		//printf("Send data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
	   	wiringPiSPIDataRW(channel, buffer, 5);
		//printf("Received data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
		speed_turret = compute_speed_turret(spi->frombytes(5, buffer)); 
		if (print) {
			printf("speed turret: %f \n", speed_turret);
		}


		// Data from left wheel speed
		buffer[0] = 0x04; 
		
		//printf("Send data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
	   	wiringPiSPIDataRW(channel, buffer, 5);
		//printf("Received data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
		speed_left_wheel = compute_speed_wheel_motor(spi->frombytes(5, buffer));
		if (print) {
			printf("speed left wheel: %f \n", speed_left_wheel);
		}

		// Data from right wheel speed
		buffer[0] = 0x05;
		
		//printf("Send data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
	   	wiringPiSPIDataRW(channel, buffer, 5);
		//printf("Received data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
		speed_right_wheel = compute_speed_wheel_motor(spi->frombytes(5, buffer));
		if (print) {
			printf("speed right wheel: %f \n", speed_right_wheel);
		}

		//TURRET TRACKING TARGET
		/////////////////////////////////////////////////////////////////////////////////////
		
		if((angle_turret - old_angle_turret) < -10 )//this means we did one turn
		{
			//we can check is something was detected by the RPI
			if(obstacle_detected_onThisTurn == 1)
			{
				target_not_found = 0;
				//if the target is   found with reset the error
				home_angularPositionPI->errorSum = 0.0;
				home_linearPositionPI->errorSum = 0.0;
				turret_speed = 14;
			}
			else
			{
				target_not_found = target_not_found + 1; //If not a counter increment in order to know when it was refresh
				if(target_not_found>not_found_max)
				{	
					//if the target is  not found with reset the error
					target_angularPositionPI->errorSum = 0.0;
					target_linearPositionPI->errorSum = 0.0;
					angleIn = 0;
					angleOut = 0;
					turret_speed = 8;
				}
			}
			obstacle_detected_onThisTurn = 0;
		}
			
		//If the target is detected we get the angleIn and Out by SPI
		if (digitalRead(laserSignal) == 0)
		{	
			obstacle_detected_onThisTurn = 1;
			obstacle_detected = 1; 

			if(obstacle_was_detected == 0)
			{
				obstacle_was_detected = 1;
			}
		}
		else
		{
			obstacle_detected = 0; 
			if(obstacle_was_detected == 1)
			{
				//obstacle_was_detected = 0;

				//"AngleIN" it is the first angle when the robot detect the beacon
				buffer[0] = 0x06;
				//printf("Send data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
	   			wiringPiSPIDataRW(channel, buffer, 5);
				//printf("Lasersync : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
				angleIn = compute_angle_turret(spi->frombytes(5, buffer));
				

				//"AngleOUT" it is the last angle when the beacon was detected.
				buffer[0] = 0x07;
				//printf("Send data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
	   			wiringPiSPIDataRW(channel, buffer, 5);
				//printf("Lasersync : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
				angleOut = compute_angle_turret(spi->frombytes(5, buffer));
				

				//Can compute the new position 
				deltaAngle = abs(angleOut - angleIn); 
			
				if(deltaAngle > 40) 
					deltaAngle = 360 - deltaAngle ;
				
				angleMes = angleIn + deltaAngle/2;
				if(compute_distance(deltaAngle/2,0) > 0)
					distanceMes = compute_distance(deltaAngle/2,0); //set to 1 to be un close mode but not use
			
				
			}
		}
		
		if(print)
		{	
			printf("target found %d \n",target_not_found);
			printf("delta angle %lf \n", deltaAngle/2);
			printf("orrientation to target %lf \n", angleIn+deltaAngle/2);
			printf("distance from target %lf \n", distanceMes);
		}
		
		
		
		//UPDATING THE ROBOT POSTION
		///////////////////////////////////////////////////////////////////////////////////////
		run_mapping(positionMap,count_L,count_R);
		print_mapping(positionMap);
		
		
		//SETTING THE SPEED IN TERMS OF THE TARGET
		///////////////////////////////////////////////////////////////////////////////////////

		//Computation for the rotation control
		// if the target is not detected than the robot try to reach a point based on his position
		
		if (onTarget ==  1 && target_not_found < not_found_max)
		{	
			if(close_to_target == 0)
			{	
				angleRef = 0;
				distanceRef = 0;
				printf("Target detected at %f degred and %f m ! On my way \n",angleMes,distanceMes);
		 		omref_rotation = run_positionRotation_controller(target_angularPositionPI,angleRef,angleMes,dt_PI);
				omref_forward = 0;
				if (angleMes  > -30.0 && angleMes  < 25.0) 
					printf("Moving to the target ... distance : %lf \n",distanceMes);
					omref_forward = -run_positionForward_controller(target_linearPositionPI,distanceRef,distanceMes,dt_PI);
				if(distanceMes < 0.3)
				{
					printf("Close enought \n");
					omref_forward = 0; 
					omref_rotation = 0;
					close_to_target = 1;
				}
			}
			if(distanceMes > 0.5)
			{
				printf("Target mooved \n",distanceMes);
				close_to_target = 0;
			}
				
	        }
		else if(backHome ==  1 && target_not_found >= not_found_max)
		{	
			printf("Nothing detected :( Going back home ... \n");
			close_to_target = 0;

			//This compute the new value of reference for the controller			
			run_trajectoryTracker(tracker); 

			//This are the two PI controller, there P infact because there is no integral factor
			omref_rotation = run_positionRotation_controller(home_angularPositionPI,tracker->angleRef*180/M_PI,tracker->angleMes*180/M_PI,dt_PI);
			omref_forward = 0;
			if (tracker->angleMes*180/M_PI  > tracker->angleRef*180/M_PI-20.0 && tracker->angleMes  < tracker->angleRef*180/M_PI+20.0*180/M_PI)
				omref_forward = -run_positionForward_controller(home_linearPositionPI,tracker->distanceRef,tracker->distanceMes,dt_PI);

			print_tracker(tracker);
			 
			//omref_forward  = 0;
			//omref_rotation = 0;
			printf("omref_forward %lf \n",omref_forward );
			printf("omref_rotation %lf \n",omref_rotation );
			
		}
		else
		{
			omref_forward  = 0;
			omref_rotation = 0;
		}
		


		double omega_max = 14;
		if(omref_forward > omega_max) 
		{
			printf("warning speed saturated to %lf rad/s \n",omega_max);
			omref_forward = omega_max;
		}
		if(omref_forward < -omega_max) 
		{
			printf("warning speed saturated to %lf rad/s \n",omega_max);
			omref_forward = omega_max;
		}
		
		omega_max = omega_max+2;
		omega_ref[0] = omref_forward+omref_rotation;
		omega_ref[1] = omref_forward-omref_rotation; 
		

		//To limit the speed max
		if(omega_ref[0] > omega_max) 
		{
			printf("warning speed saturated to %lf rad/s \n",omega_max);
			omega_ref[0] = omega_max;
		}
		if(omega_ref[1] > omega_max) 
		{
			printf("warning speed saturated to %lf rad/s \n",omega_max);
			omega_ref[1] = omega_max;
		}
		if(omega_ref[0] < -omega_max) 
		{
			printf("warning speed saturated to %lf rad/s \n",omega_max);
			omega_ref[0] = -omega_max;
		}
		if(omega_ref[1] < -omega_max)
		{
			printf("warning speed saturated to %lf rad/s \n",omega_max);
			omega_ref[1] = -omega_max;
		}
		

		theCtrlStruct->theCtrlIn->l_wheel_speed = speed_left_wheel;
		theCtrlStruct->theCtrlIn->r_wheel_speed = speed_right_wheel;


		run_speed_controller(theCtrlStruct, omega_ref); 
		//printf("L_ID: %f   &&&  R_ID: %f \n", -theCtrlStruct->theCtrlOut->wheel_commands[L_ID], theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
		if (set && setWheel) {
			can->push_PropDC(-theCtrlStruct->theCtrlOut->wheel_commands[R_ID],-theCtrlStruct->theCtrlOut->wheel_commands[L_ID]);
		}

	
		
		// Turn on the turret
		if (set && setTurret){
			can->push_TowDC(turret_speed);
		}

		can->check_receive();

		// blink the led
		if (t%1000==0){
			can->ctrl_led(1);
		}
		if (t%1000==500){
			can->ctrl_led(0);
		}


	}

	// Turn off the wheels
	can->push_PropDC(10,10);

}
