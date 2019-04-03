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
	kraken->can->ctrl_motor(1);
	kraken->can->push_PropDC(20, 20);
	
	while(true)
	{
		delay(1);  //delay for the controller
		
		
		//Data from motor encoders
		//Speed		
		buffer[0] = 0x02; // motor encoder right wheel	
	   	wiringPiSPIDataRW(channel, buffer, 5);
		kraken->motor_speed_right_wheel = compute_speed_wheel_motor(kraken->spi->frombytes(5, buffer)); 
		printf("speed  R %lf \n", kraken->motor_speed_right_wheel);
		
		buffer[0] = 0x03; // motor encoder left wheel	
	   	wiringPiSPIDataRW(channel, buffer, 5);
	   	kraken->motor_speed_left_wheel = compute_speed_wheel_motor(kraken->spi->frombytes(5, buffer));
		printf("speed  L %lf \n", kraken->motor_speed_left_wheel);
		
		kraken->theCtrlStruct->theCtrlIn->l_wheel_speed = kraken->motor_speed_left_wheel; //data  taken from the SPI
		kraken->theCtrlStruct->theCtrlIn->r_wheel_speed = kraken->motor_speed_right_wheel;
		
		omega_ref[0] = 8;
		omega_ref[1] = 8;
		
		//run_speed_controller(kraken->theCtrlStruct, omega_ref); 
		//kraken->can->push_PropDC(kraken->theCtrlStruct->theCtrlOut->wheel_commands[R_ID], kraken->theCtrlStruct->theCtrlOut->wheel_commands[L_ID]);
		kraken->can->push_PropDC(20,20);
		
		//Data from motor encoders
		//Speed		
		
	
		//Pour le moment avec les encodeurs du moteur mais à changer avec les encodeurs odométriques
		buffer[0] = 0x00; //encoder right wheel	
	   	wiringPiSPIDataRW(channel, buffer, 5);
		kraken->odometer_speed_right_wheel = compute_speed_wheel_motor(kraken->spi->frombytes(5, buffer)); 
		printf("speed  R odom %lf \n",	kraken->odometer_speed_right_wheel );
		
		buffer[0] = 0x01; //encoder left wheel	
	   	wiringPiSPIDataRW(channel, buffer, 5);
	   	kraken->odometer_speed_left_wheel = compute_speed_wheel_motor(kraken->spi->frombytes(5, buffer));
		printf("speed  L odom%lf  \n \n", 	kraken->odometer_speed_left_wheel );
		
		buffer[0] = 0x06; //ultrasonic
	   	wiringPiSPIDataRW(channel, buffer, 5);
		printf("ultrasonic %lf \n", 	kraken->spi->frombytes(5, buffer));
		buffer[0] = 0x07; //ultrasonic
	   	wiringPiSPIDataRW(channel, buffer, 5);
		printf("ultrasonic count %lf \n \n", 	kraken->spi->frombytes(5, buffer));
		
		
		//to compute where the robot is on the map
		run_mapping(kraken->currentLocation, kraken->odometer_speed_left_wheel, kraken->odometer_speed_right_wheel);
		//print_mapping(kraken->currentLocation);
		
		//to compute where the robot needs to go
		run_trajectoryTracker(kraken->tracker, omega_ref); 
	}
	
    kraken->can->push_PropDC(10,10);
	free_Kraken(kraken);
	printf("Kraken freed");
	return 0;
	
}



