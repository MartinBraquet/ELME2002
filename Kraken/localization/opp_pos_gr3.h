/*! 
 * \author Nicolas Van der Noot
 * \file opp_pos_gr3.h
 * \brief opponents position detection
 */

#ifndef _OPP_POS_EX_H_
#define _OPP_POS_EX_H_ 

#include "../CtrlStruct_gr3.h"

#if ROBOTICS_COURSE
    #include "namespace_ctrl.h"
    NAMESPACE_INIT(ctrlGr3);
#endif

/// opponents position
typedef struct OpponentsPosition
{
	double x[2]; ///< x position of opponents [m]
	double y[2]; ///< y position of opponents [m]

	int nb_opp; ///< number of opponents

} OpponentsPosition;

// function prototype
void opponents_tower(CtrlStruct *cvs);

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif

#endif
