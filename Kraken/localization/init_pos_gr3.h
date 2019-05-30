/*! 
 * \file init_pos_gr3.h
 * \brief initial position of the robot approximate guess
 */

#ifndef _INIT_POS_EX_H_
#define _INIT_POS_EX_H_ 

#include "../CtrlStruct_gr3.h"

/// robot position (to update with odometry)
typedef struct RobotPosition
{
	double x; ///< x position [m]
	double y; ///< y position [m]
	double theta; ///< robot orientation [rad]
	
	double x_lidar; ///< x position based on triangulation [m]
	double y_lidar; ///< y position based on triangulation [m]
	double theta_lidar; ///< robot orientation based on triangulation [rad]

	double last_t; ///< last time odometry was updated
	
	double pos_covariance[3][3];	///< covariance of position with order x,y,theta
	double pos_covariance_triang[3][3];	///< covariance of position due to triangulation with order x,y,theta
	
	double odo_l_wheel_last_angle; ///< last angle for odometry update
	double odo_r_wheel_last_angle;

} RobotPosition;

// function prototype
void set_init_position(int plus_or_minus, RobotPosition *rob_pos);

#endif
