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
	
	double x_odometer; ///< odometer x position [m]
	double y_odometer; ///< odometer y position [m]
	double theta_odometer; ///< odometer robot orientation [rad]
	
	double x_beacons; ///< x position based on triangulation [m]
	double y_beacons; ///< y position based on triangulation [m]
	double theta_beacons; ///< robot orientation based on triangulation [rad]

	double last_t; ///< last time odometry was updated

	double odo_l_wheel_last_angle; // last angle for odometry update
	double odo_r_wheel_last_angle;

} RobotPosition;

// function prototype
void set_init_position(int robot_id, RobotPosition *rob_pos);

#endif
