#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "Kraken.h"

int main()
{
	// printf("hello world\n");
	printf("##############################################################################################################\n");
    	printf("\t\t\tWelcome to the project of the ELME2002 class :)");
    	printf("##############################################################################################################\n");
    	printf("\t\t I'm Kraken, please take care of me !\n");
  
	
	// Initialisation the robot
	Kraken* kraken = (Kraken*) malloc(sizeof(Kraken));
	init_Kraken(kraken);
	printf("Kraken fully initialized \n");
	
	unsigned char buffer[100];
	double omega_ref[2] = {0,0}; 
	
	//setup the motor
	kraken->can->ctrl_motor(0);
	kraken->can->push_PropDC(20, 20);
	
	while(true)
	{
		delay(1);  //delay for the controller
		
		
		//Data from motor encoders
		//Speed		
		buffer[0] = 0x04; // motor encoder right wheel	
	   	wiringPiSPIDataRW(channel, buffer, 5);
		kraken->speed_right_wheel = compute_speed_wheel_motor(kraken->spi->frombytes(5, buffer)); 
		printf("speed  R %lf \n", kraken->speed_right_wheel);
		
		buffer[0] = 0x05; // motor encoder left wheel	
	   	wiringPiSPIDataRW(channel, buffer, 5);
	   	kraken->speed_left_wheel = compute_speed_wheel_motor(kraken->spi->frombytes(5, buffer));
		printf("speed  L %lf \n", kraken->speed_left_wheel);
		
		kraken->theCtrlStruct->theCtrlIn->l_wheel_speed = kraken->speed_left_wheel; //data  taken from the SPI
		kraken->theCtrlStruct->theCtrlIn->r_wheel_speed = kraken->speed_right_wheel;
		
		omega_ref[0] = 8;
		omega_ref[1] = 8;
		
		run_speed_controller(kraken->theCtrlStruct, omega_ref); 
		kraken->can->push_PropDC(kraken->theCtrlStruct->theCtrlOut->wheel_commands[R_ID], kraken->theCtrlStruct->theCtrlOut->wheel_commands[L_ID]);
		//kraken->can->push_PropDC(20,20);
		
		//Data from motor encoders
		//Speed		
		
	
		//Pour le moment avec les encodeurs du moteur mais à changer avec les encodeurs odométriques
		buffer[0] = 0x00; //encoder right wheel	
	   	wiringPiSPIDataRW(channel, buffer, 5);
		kraken->odometer_speed_right_wheel = compute_speed_wheel_motor(kraken->spi->frombytes(5, buffer)); 
		//printf("speed  R %lf \n",kraken->speed_right_wheel);
		
		buffer[0] = 0x01; //encoder left wheel	
	   	wiringPiSPIDataRW(channel, buffer, 5);
	   	kraken->odometer_speed_left_wheel = compute_speed_wheel_motor(kraken->spi->frombytes(5, buffer));
		//printf("speed  L %lf \n", kraken->speed_left_wheel);
		
		//to compute where the robot is on the map
		run_mapping(kraken->currentLocation, kraken->odometer_speed_left_wheel, kraken->odometer_speed_right_wheel);
		print_mapping(kraken->currentLocation);
		
		//to compute where the robot needs to go
		run_trajectoryTracker(kraken->tracker, omega_ref); 
	}
	
    kraken->can->push_PropDC(10,10);
	free_Kraken(kraken);
	printf("Kraken freed");
	return 0;
	
}



