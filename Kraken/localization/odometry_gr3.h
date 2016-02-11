/*! 
 * \author Nicolas Van der Noot
 * \file odometry.h
 * \brief update the robot odometry
 */

#ifndef _ODOMETRY_EX_H_
#define _ODOMETRY_EX_H_ 

#include "../CtrlStruct_gr3.h"

#if ROBOTICS_COURSE
    #include "namespace_ctrl.h"
    NAMESPACE_INIT(ctrlGr3);
#endif

// function prototype
void update_odometry(CtrlStruct *cvs);

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif

#endif
