/*! 
 * \file calibration_gr3.h
 * \brief calibration of the robot to know its accurate initial position
 */

#ifndef _CALIBRATION_GR3_H_
#define _CALIBRATION_GR3_H_ 
 
#include "../CtrlStruct_gr3.h"

#if ROBOTICS_COURSE
    #include "namespace_ctrl.h"
    NAMESPACE_INIT(ctrlGr3);
#endif

/// robot calibration
typedef struct RobotCalibration
{
	double t_flag; ///< time to save

	int flag; ///< flag for calibration

} RobotCalibration;

// function prototype
void calibration(CtrlStruct *cvs);

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif

#endif
