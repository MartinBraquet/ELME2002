#include "odometry_gr3.h"
#include "init_pos_gr3.h"
#include "../useful/useful.h"
#include <math.h>

/*! \brief update the robot odometry
 * 
 * \param[in,out] cvs controller main structure
 */
void update_odometry(CtrlStruct *cvs)
{
	// variables declaration
	CtrlIn *inputs;         ///< controller inputs
	RobotPosition *rob_pos; ///< robot position
	Robot_dimensions *robot_dims;

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;
	robot_dims = cvs->robot_dimensions;

	//printf("odo_l_wheel_last_angle: %1.3f ; inputs->odo_r_wheel_angle: %1.3f\n", rob_pos->odo_l_wheel_last_angle, inputs->odo_r_wheel_angle);

	double delta_s_l = inputs->odo_l_wheel_angle - rob_pos->odo_l_wheel_last_angle;
	double delta_s_r = inputs->odo_r_wheel_angle - rob_pos->odo_r_wheel_last_angle;
    
	double delta_s     = robot_dims->odo_wheel_radius * (delta_s_r + delta_s_l) / 2.0; 
	double delta_theta = robot_dims->odo_wheel_radius * (delta_s_r - delta_s_l) / robot_dims->wheel_axle;
	
	//printf("%f ........................\n", robot_dims->odo_wheel_radius); 

	// new odometry position
	rob_pos->x = rob_pos->x + delta_s * cos(rob_pos->theta + delta_theta / 2.0);
	rob_pos->y = rob_pos->y + delta_s * sin(rob_pos->theta + delta_theta / 2.0);
	rob_pos->theta = rob_pos->theta + delta_theta;
	limit_angle(&rob_pos->theta);

	// last update time
	rob_pos->last_t = inputs->t;
	rob_pos->odo_l_wheel_last_angle = inputs->odo_l_wheel_angle;
	rob_pos->odo_r_wheel_last_angle = inputs->odo_r_wheel_angle;
}
