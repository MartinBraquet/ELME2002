#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "CtrlStruct_gr3.h"
#include "ctrl_main_gr3.h"
#include "regulation/speed_controller_gr3.h"
#include "regulation/speed_regulation_gr3.h"
#include "localization/lidar.h"
#include "useful/mytime.h"
#include "useful/getch_keyboard.h"
#include <pthread.h>
#include <termios.h>

#define LIDAR_ENABLED 0
 
void *LIDAR_task(void *ptr) {

	printf("START: LIDAR task\n");

	CtrlStruct *cvs = (CtrlStruct*) ptr;
	CtrlIn *inputs = cvs->inputs;

	init_LIDAR(cvs);
	int i;

	while(inputs->t < 100.0) {
		printf("LIDAR loop: %d\n", i);
		get_LIDAR_data(cvs);
		i++;
	}

	free_LIDAR(cvs);

	return 0;

}


void *keyboard_task(void *ptr) {

	printf("START: keyboard\n");

	char ch;

	CtrlStruct *cvs = (CtrlStruct*) ptr;

	while((ch = getch()) != 'q'){}

	printf("KEYBOARD mode\n");

	cvs->keyboard = 1;

	while((ch = getch()) != 'q') {
		switch(ch) { // the real value
		    case '8':
		        // code for arrow up
				printf("Arrow up\n");
				speed_regulation(cvs, 5, 5);
		        break;
		    case '2':
		        // code for arrow down
				printf("Arrow down\n");
				speed_regulation(cvs, -5, -5);
		        break;
		    case '6':
		        // code for arrow right
				printf("Arrow right\n");
				speed_regulation(cvs, -4, 4);
		        break;
		    case '4':
		        // code for arrow left
				printf("Arrow left\n");
				speed_regulation(cvs, 4, -4);
		        break;
			case '5':
		        // code for rest
				printf("Stop robot\n");
				speed_regulation(cvs, 0, 0);
		        break;
		}

		cvs->can->push_PropDC(cvs->outputs->wheel_commands[R_ID], cvs->outputs->wheel_commands[L_ID]);

	}

	printf("END: keyboard\n");
	return 0;

}


void *main_task(void *ptr) {
	printf("START: main task\n");

	CtrlStruct *cvs = (CtrlStruct*) ptr;

	//setup the motor
	cvs->can->ctrl_motor(1);
	unsigned char buffer[100];

	double start_time = get_time();

	CtrlIn *inputs = cvs->inputs;
	SPI *spi = cvs->spi;

	double t;

		

	while(!cvs->keyboard) {
		t = get_time() - start_time;
		inputs->t = t;

		printf("Time: %f\n", t);

		//Data from motor encoders
		//Speed
		buffer[0] = 0x00; // motor encoder left wheel angle
	   	wiringPiSPIDataRW(channel, buffer, 5);
		inputs->motor_enc_l_wheel_angle = compute_angle_wheel_motor(spi->frombytes(5, buffer));
 		//printf("angle r %lf \n",compute_angle_wheel_motor(spi->frombytes(5, buffer)));

		buffer[0] = 0x01; // motor encoder right wheel angle
	   	wiringPiSPIDataRW(channel, buffer, 5);
		cvs->inputs->motor_enc_r_wheel_angle = compute_angle_wheel_motor(spi->frombytes(5, buffer));
		//printf("angle l %lf \n",compute_angle_wheel_motor(spi->frombytes(5, buffer)));
		
		buffer[0] = 0x02; // motor encoder left wheel speed
        wiringPiSPIDataRW(channel, buffer, 5);
        //printf("\n encoder 1 %d \n", (spi->frombytes(5, buffer))); 
                
		buffer[0] = 0x03; // motor encoder left wheel speed
        wiringPiSPIDataRW(channel, buffer, 5);
        //printf("encoder 2 %d \n \n",( spi->frombytes(5, buffer)));
                

		buffer[0] = 0x05; // motor encoder left wheel speed
	   	wiringPiSPIDataRW(channel, buffer, 5);
	   	inputs->motor_enc_l_wheel_speed = compute_speed_wheel_motor(spi->frombytes(5, buffer));
		printf("speed  L %lf \n", inputs->motor_enc_l_wheel_speed);

		buffer[0] = 0x04; // motor encoder right wheel speed
	   	wiringPiSPIDataRW(channel, buffer, 5);
		inputs->motor_enc_r_wheel_speed = compute_speed_wheel_motor(spi->frombytes(5, buffer));
		printf("speed  R %lf \n", inputs->motor_enc_r_wheel_speed);

		inputs->odo_l_wheel_speed = inputs->motor_enc_l_wheel_speed;
		inputs->odo_r_wheel_speed = inputs->motor_enc_r_wheel_speed;
		inputs->odo_l_wheel_angle = inputs->motor_enc_l_wheel_angle;
		inputs->odo_r_wheel_angle = inputs->motor_enc_r_wheel_angle;

		controller_loop(cvs);
	
		// printf("Wheel commands: %1.3f %1.3f\n", cvs->outputs->wheel_commands[R_ID], cvs->outputs->wheel_commands[L_ID]);

		cvs->can->push_PropDC(cvs->outputs->wheel_commands[R_ID], cvs->outputs->wheel_commands[L_ID]);

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
		*/


	}

	printf("END: main task\n");

	return 0;

}

int main()
{

	// Initialisation of the robot

    CtrlIn *inputs = (CtrlIn*) malloc(sizeof(CtrlIn));
    CtrlOut *outputs = (CtrlOut*) malloc(sizeof(CtrlOut));
	CtrlStruct *cvs = init_CtrlStruct(inputs, outputs);
	controller_init(cvs);
	printf("Kraken fully initialized \n");

	pthread_t keyboard_thread;
	int i_keyboard_thread = pthread_create(&keyboard_thread, NULL, keyboard_task, (void*) cvs);

	pthread_t main_thread;
	int i_main_thread = pthread_create(&main_thread, NULL, main_task, (void*) cvs);

	if (LIDAR_ENABLED) {
		pthread_t LIDAR_thread;
		int i_LIDAR_thread = pthread_create(&LIDAR_thread, NULL, LIDAR_task, (void*) cvs);
		pthread_join(LIDAR_thread, NULL); 
	}

     pthread_join(main_thread, NULL);
     pthread_join(keyboard_thread, NULL);

    controller_finish(cvs);
	free_CtrlStruct(cvs);
	printf("Kraken freed");
	return 0;

}

