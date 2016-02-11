/*! 
 * \author Nicolas Van der Noot
 * \file init_pos_gr3.h
 * \brief initial position of the robot approximate guess
 */

#ifndef _INIT_POS_EX_H_
#define _INIT_POS_EX_H_ 

#include "../CtrlStruct_gr3.h"

#if ROBOTICS_COURSE
    #include "namespace_ctrl.h"
    NAMESPACE_INIT(ctrlGr3);
#endif

/// robot position (to update with odometry)
typedef struct RobotPosition
{
	double x; ///< x position [m]
	double y; ///< y position [m]
	double theta; ///< robot orientation [rad]

	double last_t; ///< last time odometry was updated

} RobotPosition;

// function prototype
void set_init_position(int robot_id, RobotPosition *rob_pos);

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif

#endif
