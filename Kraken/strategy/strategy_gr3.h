/*! 
 * \author Nicolas Van der Noot
 * \file strategy_gr3.h
 * \brief strategy during the game
 */

#ifndef _STRATEGY_EX_H_
#define _STRATEGY_EX_H_

#include "../CtrlStruct_gr3.h"
#include <math.h>

#if ROBOTICS_COURSE
    NAMESPACE_INIT(ctrlGr3);
#endif

// strategy main states
enum {STRAT_STATE_1, STRAT_STATE_2, STRAT_STATE_3};

/// strategy
typedef struct Strategy
{
	int state; ///< main state of the strategy
	
} Strategy;

// function prototype
void main_strategy(CtrlStruct *cvs);

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif

#endif

