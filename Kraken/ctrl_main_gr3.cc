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
#include "localization/calibration_gr3.h"
#include "strategy/strategy_gr3.h"
#include "path/path_planning_gr3.h"
#include "simu_game_gr3.h"

#if ROBOTICS_COURSE
    #include "user_realtime.h"
    #include "namespace_ctrl.h"
    NAMESPACE_INIT(ctrlGr3);
#endif

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

	//Set the SPI bus
	cvs->spi;
	int fd = wiringPiSPISetup(channel, clockSpi);
	printf("SPI setup: %d \n",fd);


	// robot position
	set_init_position(cvs->robot_id, cvs->rob_pos);
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
    // printf("Control loop \n");
    
    /*
	cvs->outputs->wheel_commands[0] = cvs->py_outputs->wheel_commands[0];
	cvs->outputs->wheel_commands[1] = cvs->py_outputs->wheel_commands[1];
	cvs->outputs->tower_command = cvs->py_outputs->tower_command;
	*/
	
	printf("l_wheel_speed:%f [rad/s] ; r_wheel_speed:%f [rad/s]\n", cvs->inputs->l_wheel_speed, cvs->inputs->r_wheel_speed);
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

	// opponents position detection with the tower
	opponents_tower(cvs);

	// tower control
	outputs->tower_command = 0.0;

	switch (cvs->main_state)
	{
		case CALIB_STATE:
			calibration(cvs);
			break;

		case WAIT_INIT_STATE:
			speed_regulation(cvs, 0.0, 0.0);

			if (t > 0.0)
			{
				cvs->main_state = RUN_STATE;
			}
			break;

		case RUN_STATE:
			main_strategy(cvs);

			if (t > 89.0)
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
