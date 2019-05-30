#include "CtrlStruct_gr3.h"
#include "localization/init_pos_gr3.h"
#include "localization/odometry_gr3.h"
#include "localization/opp_pos_gr3.h"
#include "localization/lidar.h"
#include "regulation/speed_regulation_gr3.h"
#include "regulation/speed_controller_gr3.h"
#include "strategy/strategy_gr3.h"
#include "path/path_planning_gr3.h"



/*! \brief initialize the controller structure
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] outputs outputs of the controller
 * \return controller main structure
 *
 * Many parameters are set to arbitrary values, which will be corrected in 'controller_init'.
 */
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs)
{
	int i;
	CtrlStruct *cvs;

	cvs = (CtrlStruct*) malloc(sizeof(CtrlStruct));

	// io
	cvs->inputs  = inputs;
	cvs->outputs = outputs;

	cvs->keyboard = 0;
	cvs->stop_lidar = 0;

	// states
	cvs->main_state = 0;
	
	cvs->started = 0;

	// robot position
	cvs->rob_pos = (RobotPosition*) malloc(sizeof(RobotPosition));

	cvs->rob_pos->last_t = 0.0;
	
	// opponents position
	cvs->opp_pos = (OpponentsPosition*) malloc(sizeof(OpponentsPosition));

	cvs->opp_pos->nb_opp = 1;
	for(i=0; i<2; i++)
	{
		cvs->opp_pos->x[i] = 10.0;
		cvs->opp_pos->y[i] = 10.0;
	}

	// speed regulation
	cvs->sp_reg = (SpeedRegulation*) malloc(sizeof(SpeedRegulation));

	cvs->sp_reg->last_t = 0.0;

	// strategy
	cvs->strat = (Strategy*) malloc(sizeof(Strategy));
	cvs->strat->mini_state = 0;

	// path-planning
	cvs->path = (PathPlanning*) malloc(sizeof(PathPlanning));
	
	// motors
	cvs->motor_str = (MotorStruct*) malloc(sizeof(MotorStruct));
	
	// robot dimensions
	cvs->robot_dimensions = (Robot_dimensions*) malloc(sizeof(Robot_dimensions));
	
	// LIDAR data
	cvs->lidar_data = (LIDAR_data*) malloc(sizeof(LIDAR_data));
	
    cvs->robot_dimensions->radius = 0.13; // [m]
    cvs->robot_dimensions->wheel_radius = 0.025; // [m]
    cvs->robot_dimensions->odo_wheel_radius = 0.0225; // [m]
    cvs->robot_dimensions->wheel_axle = 0.322; // [m]
    cvs->robot_dimensions->lidar_distance = 0.01; // [m]
    cvs->robot_dimensions->beacon_radius = 0.04; // [m]
    cvs->robot_dimensions->microswitch_distance = 0.09; // [m]
   
    // data to save 
    cvs->save = (SaveFileStruct*) malloc(sizeof(SaveFileStruct));
    saveResults_init(cvs->save);
	return cvs;
}

/*! \brief release controller main structure memory
 * 
 * \param[in] cvs controller main structure
 */
void free_CtrlStruct(CtrlStruct *cvs)
{
	free_speed_controller(cvs);
	free(cvs->path);
	free(cvs->strat);
	free(cvs->sp_reg);
	free(cvs->opp_pos);
	free(cvs->rob_pos);
	free(cvs->motor_str);
	free(cvs->robot_dimensions);
	free(cvs->lidar_data);
	saveResults_finish(cvs->save);
	free(cvs->save);
	
	free(cvs);
	
	printf("Free CtrlStruct\n");
}

