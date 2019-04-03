/*! 
 * \file ctrl_main_gr3.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */
 
// x = 0.87 m   y = 1.4 m

#include "ctrl_main_gr3.h"
#include <stdio.h>
#include <time.h>
#include <ctime>
#include <cmath>
#include "localization/init_pos_gr3.h"
#include "localization/odometry_gr3.h"
#include "localization/opp_pos_gr3.h"
#include "regulation/speed_regulation_gr3.h"
#include "regulation/speed_controller_gr3.h"
#include "strategy/strategy_gr3.h"
#include "path/path_planning_gr3.h"
#include "simu_game_gr3.h"


/*! \brief initialize controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
void controller_init(CtrlStruct *cvs)
{
    printf("Control init \n");
    
    // variables declaration
	double t;
	CtrlIn *inputs;

	// variables initialization
	inputs = cvs->inputs;
	inputs->t = 0.0;
	t = inputs->t;

	/*
	 *  Communication channels
	 */

	//Set the GPIO from the rapsberry PI
	if(wiringPiSetup() == -1)
	{
		printf("error : unable to set the GPI from the raspberry \n");
	}
 
	//Pin reset of the FPGA at the start.
	pinMode(resetPin,OUTPUT);
	digitalWrite(resetPin,HIGH);
	delay(50);
	digitalWrite(resetPin,LOW);

	//Set the Can bus (Motor control)
	cvs->can = new CAN(CAN_BR);
	cvs->can->configure();
	cvs->can->push_PropDC(10.0, 10.0);

	//Set the SPI bus
	int fd = wiringPiSPISetup(channel, clockSpi);
	printf("SPI setup: %d \n",fd);


	// robot position
	set_init_position(cvs->robot_team, cvs->rob_pos);
	cvs->rob_pos->last_t = t;

	// speed regulation
	cvs->sp_reg->last_t = t;

	// motors initialization
	init_speed_controller(cvs);

	// Map initialization
	init_path_planning(cvs);


}

/*! \brief controller loop (called every timestep)
 * 
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs)
{
    // printf("Control loop \n");c
    
	//printf("TRIANGULATION: x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", cvs->rob_pos->x_lidar, cvs->rob_pos->y_lidar, cvs->rob_pos->theta_lidar * 180 / M_PI);
	//printf("GUESSED      : x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta * 180 / M_PI);

	//printMatrix(cvs->rob_pos->pos_covariance, 3, 3);
	//printMatrix(cvs->rob_pos->pos_covariance_triang, 3, 3);

	//printf("OPPONENTS: x:%1.5f [m] ; y:%1.5f [m]\n", cvs->opp_pos->x[0], cvs->opp_pos->y[0]);
	//printf("OPPONENTS: x:%1.5f [m] ; y:%1.5f [m]\n", cvs->opp_pos->x[1], cvs->opp_pos->y[1]);

	//printf("l_wheel_speed:%f [rad/s] ; r_wheel_speed:%f [rad/s]\n", cvs->inputs->odo_l_wheel_speed, cvs->inputs->odo_r_wheel_speed);

	// variables declaration
	double t;
	CtrlIn  *inputs;
	CtrlOut *outputs; 

	// variables initialization
	inputs  = cvs->inputs;
	outputs = cvs->outputs;

	t = inputs->t;

	// update the robot odometry
	update_odometry(cvs);
	
	//printf("main_state: %d\n", cvs->main_state);

	switch (cvs->main_state)
	{
		case WAIT_INIT_STATE:
			speed_regulation(cvs, 0.0, 0.0);

			if (t > 3.0)
			{
				cvs->main_state = RUN_STATE;
			}
			break;

		case RUN_STATE:
			main_strategy(cvs);

			if (t > 95.0)
			{
				cvs->main_state = STOP_END_STATE;
			}
			break;

		case STOP_END_STATE:
			speed_regulation(cvs, 0.0, 0.0);
			break;
	
		default:
			printf("Main loop error:unknown state : %d !\n", cvs->main_state);
			exit(EXIT_FAILURE);
	}

}

/*! \brief last controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{
    printf("Control finish \n");
}

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif
