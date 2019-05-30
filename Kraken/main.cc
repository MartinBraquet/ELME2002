#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "CtrlStruct_gr3.h"
#include "ctrl_main_gr3.h"
#include "regulation/speed_controller_gr3.h"
#include "regulation/speed_regulation_gr3.h"
#include "localization/triangulation_gr3.h"
#include "localization/init_pos_gr3.h"
#include "localization/opp_pos_gr3.h"
#include "localization/init_pos_gr3.h"
#include "localization/odometry_gr3.h"
#include "localization/lidar.h"
#include "localization/sensor_merging_gr3.h"
#include "useful/mytime.h"
#include "useful/getch_keyboard.h"
#include "IO/dynamixel.h"
#include "IO/pneumatics.h"
#include "IO/server.h"
#include "IO/client.h"
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <SDL2/SDL.h>

#define LIDAR_ENABLED 1
#define KEYBOARD_ENABLED 1
#define ANDROID_ENABLED 0
#define JUMPER_ENABLED 1
#define ACTIONS_ENABLED 1
#define KEYBOARD_COMMAND 10.0
#define PRINT_FPGA 0
#define USE_LT24 1

void get_FPGA_data(CtrlIn *inputs, unsigned char *buffer, SPI *spi)
{
	//Data from motor encoders

	buffer[0] = 0x00; // motor encoder left wheel angle
	wiringPiSPIDataRW(channel, buffer, 5);
	inputs->motor_enc_l_wheel_angle = compute_angle_wheel_motor(spi->frombytes(5, buffer));
#if PRINT_FPGA
	printf("motor_enc_l_wheel_angle: %lf \n", inputs->motor_enc_l_wheel_angle);
#endif

	buffer[0] = 0x01; // motor encoder right wheel angle
	wiringPiSPIDataRW(channel, buffer, 5);
	inputs->motor_enc_r_wheel_angle = -compute_angle_wheel_motor(spi->frombytes(5, buffer));
#if PRINT_FPGA
	printf("motor_enc_r_wheel_angle: %lf \n", inputs->motor_enc_r_wheel_angle);
#endif

	buffer[0] = 0x03; // odometer encoder left wheel angle
	wiringPiSPIDataRW(channel, buffer, 5);
	inputs->odo_l_wheel_angle = compute_angle_wheel_odo(spi->frombytes(5, buffer));
#if PRINT_FPGA
	printf("odo_l_wheel_angle: %lf \n", inputs->odo_l_wheel_angle);
#endif

	buffer[0] = 0x02; // odometer encoder right wheel angle
	wiringPiSPIDataRW(channel, buffer, 5);
	inputs->odo_r_wheel_angle = compute_angle_wheel_odo(spi->frombytes(5, buffer));
#if PRINT_FPGA
	printf("odo_r_wheel_angle: %lf \n", inputs->odo_r_wheel_angle);
#endif

	buffer[0] = 0x04; // motor encoder left wheel speed
	wiringPiSPIDataRW(channel, buffer, 5);
	inputs->motor_enc_l_wheel_speed = compute_speed_wheel_motor(spi->frombytes(5, buffer));
#if PRINT_FPGA
	printf("motor_enc_l_wheel_speed: %lf \n", inputs->motor_enc_l_wheel_speed);
#endif

	buffer[0] = 0x05; // motor encoder right wheel speed
	wiringPiSPIDataRW(channel, buffer, 5);
	inputs->motor_enc_r_wheel_speed = -compute_speed_wheel_motor(spi->frombytes(5, buffer));
#if PRINT_FPGA
	printf("motor_enc_r_wheel_speed: %lf \n", inputs->motor_enc_r_wheel_speed);
#endif
/*
	buffer[0] = 0x07; // odometer encoder left wheel speed
	wiringPiSPIDataRW(channel, buffer, 5);
	inputs->odo_l_wheel_speed = -compute_speed_wheel_odo(spi->frombytes(5, buffer));
#if PRINT_FPGA
	printf("odo_l_wheel_speed: %lf \n", inputs->odo_l_wheel_speed);
#endif

	buffer[0] = 0x06; // odometer encoder right wheel speed
	wiringPiSPIDataRW(channel, buffer, 5);
	inputs->odo_r_wheel_speed = compute_speed_wheel_odo(spi->frombytes(5, buffer));
#if PRINT_FPGA
	printf("odo_r_wheel_speed: %lf \n", inputs->odo_r_wheel_speed);
#endif
*/
	buffer[0] = 0x08; // start PI
	wiringPiSPIDataRW(channel, buffer, 5);
	inputs->start_RPI = spi->frombytes(5, buffer);
#if PRINT_FPGA
	printf("start_RPI: %d \n\n", inputs->start_RPI);
#endif



	/*
		//data ultrasonic sensor
		buffer[0] = 0x06; // motor encoder right wheel speed
		wiringPiSPIDataRW(channel, buffer, 5);
		printf("U1 %lf \n",(spi->frombytes(5, buffer)));

		buffer[0] = 0x07; // motor encoder right wheel speed
		wiringPiSPIDataRW(channel, buffer, 5);
		printf("U2 %lf \n",(spi->frombytes(5, buffer)));

		buffer[0] = 0x08; // motor encoder right wheel speed
		wiringPiSPIDataRW(channel, buffer, 5);
		printf("U3 %lf \n",(spi->frombytes(5, buffer)));

		buffer[0] = 0x09; // motor encoder right wheel speed
		wiringPiSPIDataRW(channel, buffer, 5);
		printf("U4 %lf \n \n",(spi->frombytes(5, buffer)));
	*/
}


void set_FPGA_data(CtrlStruct *cvs, unsigned char *buffer)
{
	CtrlOut *outputs = cvs->outputs;

	//Data for pneumatic commands
	buffer[4] = outputs->pneumatic_commands;
	buffer[0] = 0x84;
	wiringPiSPIDataRW(channel, buffer, 5);
	#if PRINT_FPGA
		printf("pneumatic_commands: %X \n\n", outputs->pneumatic_commands);
	#endif
}

void *LIDAR_task(void *ptr) {

	printf("START: LIDAR task\n");

	CtrlStruct *cvs = (CtrlStruct*) ptr;
	CtrlIn *inputs = cvs->inputs;

	while(!cvs->stop_lidar) {
		//printf("LIDAR loop\n");
		//double lidar_time = get_time();
		get_LIDAR_data(cvs);
		//lidar_time = get_time() - lidar_time;
		//printf("lidar_time: %f\n", lidar_time);
		printf("t: %f\n", cvs->inputs->t);
		triangulation_mean_data(cvs);
		//triangulation(cvs);
		//if (cvs->inputs->t < 100.0) 
		//	continue;
		//printf("TRIANGULATION      : x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", cvs->rob_pos->x_lidar, cvs->rob_pos->y_lidar, cvs->rob_pos->theta_lidar * 180 / M_PI);
		//mergeSensorData(cvs);
		//triangulation_distances_all_three(cvs);
		//triangulation_distances_two_nearest(cvs);
		//printf("GUESSED      : x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta * 180 / M_PI);
		Kalman_filter(cvs);
	}

	free_LIDAR(cvs);

	return 0;

}

/*
void *web_task(void *ptr) {

	printf("START: WEB task\n");

	CtrlStruct *cvs = (CtrlStruct*) ptr;
	CtrlIn *inputs = cvs->inputs;

	

	return 0;

}
*/

void *keyboard_task(void *ptr) {

	printf("START: keyboard\n");

	CtrlStruct *cvs = (CtrlStruct*) ptr;
	unsigned char buffer[100];
	SPI *spi = cvs->spi;
	
	sem_t *actions_semaphore;
	actions_semaphore = sem_open("/actions_semaphore", O_CREAT);

	double start_time = get_time();
	
	SDL_Init(SDL_INIT_VIDEO);
    SDL_Window * window = SDL_CreateWindow("Kraken - ELME2002", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 320, 240, 0);

	printf("KEYBOARD mode\n");

    SDL_Event event;  
    int quit;
    double V = 0.0;
    double W = 0.0;
    
    quit = 0;
    
    while(quit != 2 && cvs->keyboard < 2){
        while(SDL_PollEvent(&event)){
            switch(event.type)
            {
                case SDL_KEYDOWN:
                    switch(event.key.keysym.sym)
                    {
                        case SDLK_ESCAPE:
                            quit = 1;
                            break;
                        case SDLK_UP:
							V = KEYBOARD_COMMAND;
                            break;
                        case SDLK_DOWN:
							V = -KEYBOARD_COMMAND;   
                            break;
                        case SDLK_RIGHT:
							W = - KEYBOARD_COMMAND / 2.0;
                            break;
                        case SDLK_LEFT:
							W = KEYBOARD_COMMAND / 2.0;
                            break;
                        case SDLK_q:
                            cvs->keyboard++;
                            quit++;
                            break;
						case SDLK_d:
							printf("testing dynamixel\n");
							dynamixelSetGoalPosition(0xfe, 0x150);
							sleep(2);
							dynamixelSetGoalPosition(0xfe, 0x001);
							sleep(2);
							break;
						case SDLK_0:
							printf("reset pneumatics 1\n");
							buffer[4] = 0;
							buffer[0] = 0x84;
							wiringPiSPIDataRW(channel, buffer, 5);
							break;
						case SDLK_p:
							printf("set all pneumatics\n");
							buffer[4] = 0xff;
							buffer[0] = 0x84;
							wiringPiSPIDataRW(channel, buffer, 5);
							break;
						case SDLK_1:
							printf("void pump 1\n");
							buffer[4] = VOID_PUMP_1_MASK;
							buffer[0] = 0x84;
							wiringPiSPIDataRW(channel, buffer, 5);
							break;
						case SDLK_2:
							printf("void pump 2\n");
							buffer[4] = VOID_PUMP_2_MASK;
							buffer[0] = 0x84;
							wiringPiSPIDataRW(channel, buffer, 5);
							break;
						case SDLK_3:
							printf("piston 1\n");
							buffer[4] = PISTON_1_MIDDLE_MASK;
							buffer[0] = 0x84;
							wiringPiSPIDataRW(channel, buffer, 5);
							break;
						case SDLK_4:
							printf("piston 2\n");
							buffer[4] = PISTON_1_FULL_MASK;
							buffer[0] = 0x84;
							wiringPiSPIDataRW(channel, buffer, 5);
							break;
						case SDLK_5:
							printf("piston 3\n");
							buffer[4] = PISTON_2_MASK;
							buffer[0] = 0x84;
							wiringPiSPIDataRW(channel, buffer, 5);
							break;
						case SDLK_9:
							printf("action 1\n");
							cvs->outputs->next_action = 1;
							sem_post(actions_semaphore);
							break;
						case SDLK_8:
							printf("action 2\n");
							cvs->outputs->next_action = 2;
							sem_post(actions_semaphore);
							break;
						case SDLK_7:
							printf("action 3\n");
							cvs->outputs->next_action = 3;
							sem_post(actions_semaphore);
							break;
						case SDLK_6:
							printf("action 4\n");
							cvs->outputs->next_action = 4;
							sem_post(actions_semaphore);
							break;
						case SDLK_l:
							printf("action 5\n");
							cvs->outputs->next_action = 5;
							sem_post(actions_semaphore);
							break;
						case SDLK_k:
							printf("action 6\n");
							cvs->outputs->next_action = 6;
							sem_post(actions_semaphore);
							break;
						case SDLK_j:
							printf("action 7\n");
							cvs->outputs->next_action = 7;
							sem_post(actions_semaphore);
							break;
						case SDLK_h:
							printf("action 10\n");
							cvs->outputs->next_action = 10;
							sem_post(actions_semaphore);
							break;

						default : break;
                    }
                    break;
                case SDL_KEYUP:
                    switch(event.key.keysym.sym)
                    {
                        case SDLK_UP:
                            V = 0;
                            break;
                        case SDLK_DOWN:
                            V = 0;     
                            break;
                        case SDLK_RIGHT:
                            W = 0;
                            break;
                        case SDLK_LEFT:
                            W = 0;
                            break;
                        default : break;
                    }
                    break;
                default : break;
		    }
			//printf("V_l = %lf   V_r = %lf\n", V + W, V - W);
        }
		if (quit == 1) {	
			cvs->inputs->t = get_time() - start_time;
			get_FPGA_data(cvs->inputs, buffer, spi);
			update_odometry(cvs);
			speed_regulation(cvs, V + W, V - W);
			//printf("GUESSED      : x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta * 180 / M_PI);
			//printf("Wheel commands: %1.3f %1.3f\n", cvs->outputs->wheel_commands[R_ID], cvs->outputs->wheel_commands[L_ID]);
			cvs->can->push_PropDC(10+cvs->outputs->wheel_commands[R_ID], 10-cvs->outputs->wheel_commands[L_ID]);
		}
    }

	printf("END: keyboard\n");
	
	cvs->stop_lidar = 1;
	
#if ANDROID_ENABLED
	if (cvs->keyboard == 2)
		exit(0);
#endif
	
	return 0;

}

#if ACTIONS_ENABLED
void* actions_task(void *ptr) {
	sem_t *actions_semaphore;
	
	actions_semaphore = sem_open("/actions_semaphore", O_CREAT);
	sem_init(actions_semaphore, 0, 1);
	struct timespec ts;
	int sem_status;


	CtrlStruct *cvs = (CtrlStruct*) ptr;
	unsigned char buffer[100];
	SPI *spi = cvs->spi;
	
	unsigned char *action_in_progress = &cvs->outputs->action_in_progress;

	printf("START: actions task\n");

	static int action_state = 0;
	*action_in_progress = 0;
	
	while (cvs->keyboard < 2) {
		
		// cvs->outputs->next_action = 25;
		clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_sec += 3;
		sem_status = sem_timedwait(actions_semaphore, &ts);
		
		printf("action: %d      action_state: %d\n", cvs->outputs->next_action, action_state);
		
		if (cvs->outputs->next_action == 25) continue;
		
//		if (cvs->outputs->next_action != 25)
//			*action_in_progress = 1;
		
		switch (cvs->outputs->next_action)
		{
		case 1:
				*action_in_progress = 1;
				cvs->outputs->pneumatic_commands = VOID_PUMP_1_MASK | PISTON_1_FULL_MASK;
				set_FPGA_data(cvs, buffer);
				printf("action: %d\n", cvs->outputs->next_action);
				
				usleep(2500000);
				
				cvs->outputs->pneumatic_commands = VOID_PUMP_1_MASK | PISTON_1_MIDDLE_MASK;
				set_FPGA_data(cvs, buffer);
				
				usleep(1000000);
				
				*action_in_progress = 0;
				
				cvs->outputs->pneumatic_commands = PISTON_1_MIDDLE_MASK;
				set_FPGA_data(cvs, buffer);
				
				usleep(2000000);
				
				
				cvs->outputs->pneumatic_commands = 0;
				set_FPGA_data(cvs, buffer);
				
				cvs->outputs->next_action = 0;
				break;
//			case 3:	
//				// approach wall
//				speed_regulation(cvs, 2.5, 2.5);
//				usleep(500000);
//				speed_regulation(cvs, 0, 0);
//				
//				action_state=0;
//				*action_in_progress = 0;
//				break;

		case 2:
				// advance both pistons 1st stage
				*action_in_progress = 1;
				
				cvs->outputs->pneumatic_commands = VOID_PUMP_1_MASK | PISTON_1_FULL_MASK;
				set_FPGA_data(cvs, buffer);
				usleep(2000000);

				// retract both pistons 1st stage, and keep holding them
				cvs->outputs->pneumatic_commands = VOID_PUMP_1_MASK;
				set_FPGA_data(cvs, buffer);
				usleep(2000000);
				
				dynamixelSetGoalPosition(DYNAMIXEL_ATOM_PUSHER_ID, 0x200);
				
				*action_in_progress = 0;
				cvs->outputs->next_action = 0;
				break;

		case 3:
				// move bottom to top

				*action_in_progress = 1;
				
				dynamixelSetWheelMode(DYNAMIXEL_ELEVATOR_ID);
				dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID,0x3ff+0x400);
				printf("action: %d\n", cvs->outputs->next_action);
				usleep(4.4*1000000);
				dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID,0);

				// advance piston 2nd stage
				cvs->outputs->pneumatic_commands = VOID_PUMP_1_MASK | PISTON_2_MASK;
				set_FPGA_data(cvs, buffer);
				usleep(2000000);

				// move top to botom
				dynamixelSetWheelMode(DYNAMIXEL_ELEVATOR_ID);
				dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID,0x3ff);
				usleep(3.55*1000000);
				dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID,0);

				// restract piston 2nd stage
				cvs->outputs->pneumatic_commands = VOID_PUMP_1_MASK;
				set_FPGA_data(cvs, buffer);				

				cvs->outputs->next_action=0;
				*action_in_progress = 0;
				
				break;

		case 4:

				// advance short piston of 1st stage and turn off pump
				*action_in_progress = 1;
				
				cvs->outputs->pneumatic_commands = PISTON_1_MIDDLE_MASK;
				set_FPGA_data(cvs, buffer);
				printf("action: %d\n", cvs->outputs->next_action);
				usleep(2000000);

				// retract short piston of 1st stage
				cvs->outputs->pneumatic_commands = 0;
				set_FPGA_data(cvs, buffer);
				usleep(1000000);

				// move bottom to top
				dynamixelSetWheelMode(DYNAMIXEL_ELEVATOR_ID);
				dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID, 0x3ff + 0x400);
				usleep(3.8 * 1000000);
				dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID, 0);

				// advance piston 2nd stage
				cvs->outputs->pneumatic_commands = PISTON_2_MASK;
				set_FPGA_data(cvs, buffer);
				usleep(2000000);

				// retract piston 2nd stage
				cvs->outputs->pneumatic_commands = 0;
				set_FPGA_data(cvs, buffer);
				usleep(2000000);

				// move top to botom
				dynamixelSetWheelMode(DYNAMIXEL_ELEVATOR_ID);
				usleep(50000);
				dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID, 0x3ff);
				usleep(3.55 * 1000000);
				dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID, 0);

				*action_in_progress = 0;
				cvs->outputs->next_action = 0;
				break;

		case 5:

				*action_in_progress = 1;
				
				dynamixelSetGoalPosition(DYNAMIXEL_ATOM_PUSHER_ID, 0x200);
				printf("action: %d\n", cvs->outputs->next_action);
				usleep(500000);

				action_state = 0;
				*action_in_progress = 0;
				cvs->outputs->next_action = 0;
				break;
		
		
		case 6:

				*action_in_progress = 1;
				
				dynamixelSetGoalPosition(DYNAMIXEL_ATOM_PUSHER_ID, 0x90);
				printf("action: %d\n", cvs->outputs->next_action);
				usleep(2000000);

				action_state = 0;
				*action_in_progress = 0;
				cvs->outputs->next_action = 0;
				break;

		case 7:

				*action_in_progress = 1;
				
				cvs->outputs->pneumatic_commands = PISTON_2_MASK | VOID_PUMP_2_MASK;
				set_FPGA_data(cvs, buffer);
				printf("action: %d\n", cvs->outputs->next_action);
				usleep(2000000);

				cvs->outputs->pneumatic_commands = VOID_PUMP_2_MASK;
				set_FPGA_data(cvs, buffer);

				*action_in_progress = 0;
				cvs->outputs->next_action = 0;
				break;

		case 8:

				*action_in_progress = 1;
				
				cvs->outputs->pneumatic_commands = PISTON_2_MASK;
				set_FPGA_data(cvs, buffer);
				printf("action: %d\n", cvs->outputs->next_action);
				usleep(2000000);

				cvs->outputs->pneumatic_commands = 0;
				set_FPGA_data(cvs, buffer);

				*action_in_progress = 0;
				cvs->outputs->next_action = 0;
				break;

		case 9:

				// advance short piston of 1st stage and turn off pump
				*action_in_progress = 1;
				
				cvs->outputs->pneumatic_commands = PISTON_1_FULL_MASK;
				set_FPGA_data(cvs, buffer);
				printf("action: %d\n", cvs->outputs->next_action);
				usleep(4000000);

				// retract short piston of 1st stage
				cvs->outputs->pneumatic_commands = 0;
				set_FPGA_data(cvs, buffer);

				// move bottom to top
				dynamixelSetWheelMode(DYNAMIXEL_ELEVATOR_ID);
				dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID, 0x3ff + 0x400);
				usleep(1.7 * 1000000);
				dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID, 0);

				// advance piston 2nd stage
				cvs->outputs->pneumatic_commands = PISTON_1_FULL_MASK;
				set_FPGA_data(cvs, buffer);
				
				usleep(2000000);

				// move top to botom
				dynamixelSetWheelMode(DYNAMIXEL_ELEVATOR_ID);
				usleep(50000);
				dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID, 0x3ff);
				usleep(1.7 * 1000000);
				dynamixelSetVelocity(DYNAMIXEL_ELEVATOR_ID, 0);
				
				dynamixelSetGoalPosition(DYNAMIXEL_ATOM_PUSHER_ID, 0x90);
				printf("action: %d\n", cvs->outputs->next_action);
				
				
				*action_in_progress = 0;

				*action_in_progress = 0;
				cvs->outputs->next_action = 0;
				break;
				
			case 10:
				cvs->outputs->pneumatic_commands = VOID_PUMP_1_MASK | PISTON_1_FULL_MASK;
				set_FPGA_data(cvs, buffer);
				usleep(2000000);
				
				cvs->outputs->pneumatic_commands = VOID_PUMP_1_MASK | PISTON_1_MIDDLE_MASK;
				set_FPGA_data(cvs, buffer);
				usleep(2000000);

				cvs->outputs->pneumatic_commands = PISTON_1_MIDDLE_MASK;
				set_FPGA_data(cvs, buffer);
				usleep(3000000);

				cvs->outputs->pneumatic_commands = 0;
				set_FPGA_data(cvs, buffer);
//				usleep(1000000);

				*action_in_progress = 0;
				cvs->outputs->next_action = 0;
				break;
	
		default:
			break;
		
		}

	}

	sem_close(actions_semaphore);
	printf("END: actions task\n");
	return 0;
}
#endif

#if ANDROID_ENABLED
void *android_task(void *ptr) {
	printf("START: android task\n");

	CtrlStruct *cvs = (CtrlStruct*) ptr;
	double V, W;
	unsigned char buffer[100];
	SPI *spi = cvs->spi;
	
	int res = 0;

	while(res == 0) {

		res = android_app(cvs, &V, &W, buffer);
		
		printf("%d \n", cvs->keyboard );
		if (cvs->keyboard == 2) {
			return 0;
		}

	}
		
	cvs->keyboard = 3;
	
	while(res == 1) {

		res = android_app(cvs, &V, &W, buffer);
		
		cvs->inputs->t = get_time();
		get_FPGA_data(cvs->inputs, buffer, spi);
		update_odometry(cvs);
		speed_regulation(cvs, V + W, V - W);
		//printf("Regul: %f %f \n", V + W, V - W);
		//printf("GUESSED      : x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta * 180 / M_PI);
		//printf("Wheel commands: %1.3f %1.3f\n", cvs->outputs->wheel_commands[R_ID], cvs->outputs->wheel_commands[L_ID]);
		cvs->can->push_PropDC(10+cvs->outputs->wheel_commands[R_ID], 10-cvs->outputs->wheel_commands[L_ID]);
		set_FPGA_data(cvs, buffer);
	}
	
	cvs->keyboard = 4;

	printf("END: android task\n");

	return 0;

}
#endif

void *main_task(void *ptr) {
	printf("START: main task\n");

	CtrlStruct *cvs = (CtrlStruct*) ptr;

	//setup the motor
	cvs->can->ctrl_motor(1);
	unsigned char buffer[100];

	CtrlIn *inputs = cvs->inputs;
	SPI *spi = cvs->spi;

	double start_time = get_time();
	double t;


	

	while(cvs->keyboard == 0) {
		t = get_time() - start_time;
		inputs->t = t;
		printf("Time: %f\n", t);
		
		get_FPGA_data(cvs->inputs, buffer, spi);
		
		controller_loop(cvs);
		
		cvs->can->push_PropDC(10+cvs->outputs->wheel_commands[R_ID], 10-cvs->outputs->wheel_commands[L_ID]);
	}

	printf("END: main task\n");

	return 0;

}

void *LT24_task(void *ptr) {

	CtrlStruct *cvs = (CtrlStruct*) ptr;

	printf("START: LT24 task\n");
	
	while (cvs->keyboard < 2) {
		RW_data_LT24(cvs, 0);
		usleep(3000000);
	}

	return 0;

}

int main(int argc, const char * argv[])
{
	

	// Initialisation of the robot

	CtrlIn *inputs = (CtrlIn*) malloc(sizeof(CtrlIn));
	CtrlOut *outputs = (CtrlOut*) malloc(sizeof(CtrlOut));
	CtrlStruct *cvs = init_CtrlStruct(inputs, outputs);
	controller_init(cvs);
	printf("Kraken fully initialized \n");
	unsigned char buffer[100];
	SPI *spi = cvs->spi;
	
	if (argc < 2) {
		printf("Enter yellow or purple\sn");
		exit(0);
	}
	if (strcmp("yellow", argv[1]) == 0) {
		cvs->robot_team = TEAM_YELLOW;
		cvs->plus_or_minus = 1;
		cvs->opp_pos->nb_opp = 1;
	} else if (strcmp("purple", argv[1]) == 0) {
		cvs->robot_team = TEAM_PURPLE;
		cvs->plus_or_minus = -1;
	} else {
		printf("Wrong param\n");
		exit(0);
	}
	
	cvs->opp_pos->nb_opp = 1;
	
	if (strcmp("0", argv[2]) == 0) {
		cvs->main_strategy = 0;
	} else if (strcmp("1", argv[2]) == 0) {
		cvs->main_strategy = 1;
	} else if (strcmp("2", argv[2]) == 0) {
		cvs->main_strategy = 2;
	} else {
		printf("Wrong param\n");
		exit(0);
	}
	
	
	cvs->outputs->pneumatic_commands = 0;
	set_FPGA_data(cvs, buffer);
	
	#if LIDAR_ENABLED
		init_LIDAR(cvs);
	#endif
	
	#if JUMPER_ENABLED
		while (inputs->start_RPI == 0) {
			get_FPGA_data(inputs, buffer, spi);
			printf("Waiting for the jumper to be pulled out of the interface card\n");
		}
	#endif

	
	cvs->outputs->score = 60;
	
	#if USE_LT24
		pthread_t LT24_thread;
		pthread_create(&LT24_thread, NULL, LT24_task, (void*) cvs);
	#endif
	

	
	printf("robot_team: %d   +/-: %d    nb_opp: %d   strat: %d\n", cvs->robot_team, cvs->plus_or_minus , cvs->opp_pos->nb_opp, cvs->main_strategy);
	set_init_position(cvs->plus_or_minus, cvs->rob_pos);
	init_path_planning(cvs);
	
	
	
	if (cvs->robot_team == TEAM_YELLOW) {
	    cvs->plus_or_minus = -1;
	} else {
	    cvs->plus_or_minus = 1;
	}

	#if KEYBOARD_ENABLED
		pthread_t keyboard_thread;
		pthread_create(&keyboard_thread, NULL, keyboard_task, (void*) cvs);
	#endif
	
	#if ANDROID_ENABLED
		pthread_t android_thread;
		pthread_create(&android_thread, NULL, android_task, (void*) cvs);
	#endif

	pthread_t main_thread;
	pthread_create(&main_thread, NULL, main_task, (void*) cvs);

	#if ACTIONS_ENABLED
		pthread_t actions_thread;
		pthread_create(&actions_thread, NULL, actions_task, (void*)cvs);
	#endif

	//pthread_t web_thread;
	//pthread_create(&web_thread, NULL, web_task, (void*) cvs);

	#if LIDAR_ENABLED
		pthread_t LIDAR_thread;
		pthread_create(&LIDAR_thread, NULL, LIDAR_task, (void*) cvs);
		pthread_join(LIDAR_thread, NULL); 
	#endif

	#if ACTIONS_ENABLED
		pthread_join(actions_thread, NULL);
	#endif
	
	pthread_join(main_thread, NULL);
	
	#if KEYBOARD_ENABLED
		pthread_join(keyboard_thread, NULL);
	#endif
	
	#if ANDROID_ENABLED
		pthread_join(android_thread, NULL);
	#endif

	cvs->outputs->pneumatic_commands = 0x00;
	set_FPGA_data(cvs, buffer);
	delay(100);

	controller_finish(cvs);
	free_CtrlStruct(cvs);
	printf("Kraken freed\n");
	
	


	return 0;

}

