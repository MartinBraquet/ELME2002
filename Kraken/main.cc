#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "ctrl_main_gr3.h"
#include "CtrlStruct_gr3.h"
#include "regulation/speed_controller_gr3.h"

int main()
{
	/*
	printf("hello world\n");
	printf("##############################################################################################################\n");
    	printf("\t\t\tWelcome to the project of the ELME2002 class :)");
    	printf("##############################################################################################################\n");
    	printf("\t\t I'm Kraken, please take care of me !\n");
  	*/

	// Initialisation the robot
        CtrlIn *inputs = (CtrlIn*) malloc(sizeof(CtrlIn));
        CtrlOut *outputs = (CtrlOut*) malloc(sizeof(CtrlOut));
	CtrlStruct *cvs = init_CtrlStruct(inputs, outputs);
	controller_init(cvs);
	printf("Kraken fully initialized \n");

	unsigned char buffer[100];

	//setup the motor
	cvs->can->ctrl_motor(1);

	while(true)
	{
		delay(1);  //delay for the controller


		//Data from motor encoders
		//Speed
		buffer[0] = 0x04; //encoder left wheel
	   	wiringPiSPIDataRW(channel, buffer, 5);
		cvs->inputs->r_wheel_speed = compute_speed_wheel_motor(cvs->spi->frombytes(5, buffer));
		printf("vitesse  R %lf \n", cvs->inputs->r_wheel_speed);

		buffer[0] = 0x05; //encoder left wheel
	   	wiringPiSPIDataRW(channel, buffer, 5);
	   	cvs->inputs->l_wheel_speed = compute_speed_wheel_motor(cvs->spi->frombytes(5, buffer));
		printf("vitesse  L %lf \n", cvs->inputs->l_wheel_speed);

		controller_loop(cvs);
		cvs->can->push_PropDC(cvs->outputs->wheel_commands[R_ID], cvs->outputs->wheel_commands[L_ID]);
		//robot->can->push_PropDC(20,20);

		//Data from motor encoders
		//Speed
		/*

		//Pour le moment avec les encodeurs du moteur mais à changer avec les encodeurs odométriques
		buffer[0] = 0x00; //encoder left wheel
	   	wiringPiSPIDataRW(channel, buffer, 5);
		robot->angle_right_wheel = compute_speed_wheel_motor(robot->spi->frombytes(5, buffer));
		//printf("vitesse  R %lf \n",robot->speed_right_wheel);

		buffer[0] = 0x01; //encoder left wheel
	   	wiringPiSPIDataRW(channel, buffer, 5);
	   	robot->angle_left_wheel = compute_speed_wheel_motor(robot->spi->frombytes(5, buffer));
		//printf("vitesse  L %lf \n",robot->speed_left_wheel);

		//to compute where the robot is on the map
		run_mapping(robot->currentLocation,robot->angle_left_wheel,robot->angle_right_wheel);
		print_mapping(robot->currentLocation);

		//to compute where the robot needs to go
		run_trajectoryTracker(robot->tracker,omega_ref); */
	}


        controller_finish(cvs);
	free_CtrlStruct(cvs);
	printf("Kraken freed");
	return 0;

}
