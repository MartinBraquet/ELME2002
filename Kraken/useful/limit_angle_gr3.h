/*! 
 * \author Nicolas Van der Noot
 * \file limit_angle_gr3.h
 * \brief limit an angle in ]-pi;pi]
 */

#ifndef _LIMIT_ANGLE_EX_H_
#define _LIMIT_ANGLE_EX_H_

#if ROBOTICS_COURSE
    #include "namespace_ctrl.h"
    NAMESPACE_INIT(ctrlGr3);
#endif

// function prototype
double limit_angle(double x);

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif

#endif
