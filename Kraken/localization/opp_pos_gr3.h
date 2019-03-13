/*! 
 * \author Nicolas Van der Noot
 * \file opp_pos_gr3.h
 * \brief opponents position detection
 */

#ifndef _OPP_POS_EX_H_
#define _OPP_POS_EX_H_ 

#include "../CtrlStruct_gr3.h"

/// opponents position
typedef struct OpponentsPosition
{
	double x[2]; ///< x position of opponents [m]
	double y[2]; ///< y position of opponents [m]
	
	int nb_opp;

} OpponentsPosition;


#endif
