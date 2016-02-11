/*! 
 * \file speed_regulation_gr3.h
 * \brief speed regulation to track wheel speed references
 */

#ifndef _SPEED_REGULATION_GR3_H_
#define _SPEED_REGULATION_GR3_H_ 
 
#include "../CtrlStruct_gr3.h"

#if ROBOTICS_COURSE
    #include "namespace_ctrl.h"
    NAMESPACE_INIT(ctrlGr3);
#endif

/// speed regulation
typedef struct SpeedRegulation
{
	double last_t; ///< last time the speed regulation was updated
	
} SpeedRegulation;

// function prototype
void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref);

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif

#endif
