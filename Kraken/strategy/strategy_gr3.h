/*! 
 * \file strategy_gr3.h
 * \brief strategy during the game
 */

#ifndef _STRATEGY_EX_H_
#define _STRATEGY_EX_H_

#include "../CtrlStruct_gr3.h"
#include <math.h>


// strategy main states
enum {STRAT_STATE_1, STRAT_STATE_2, STRAT_STATE_3, STRAT_STATE_4};

enum {MINI_STRAT_STATE_1, MINI_STRAT_STATE_2, MINI_STRAT_STATE_3};
enum {MINI_STRAT_STRAIGHT_LINE, MINI_STRAT_ROTATE_LEFT, MINI_STRAT_STRAIGHT_LINE_BACK, MINI_STRAT_STRAIGHT_LINE_TOP, MINI_STRAT_ROTATE_RIGHT, MINI_STRAT_ROTATE_RIGHT2, MINI_STRAT_WAIT_END};

/// strategy
typedef struct Strategy
{
	int state; ///< main state of the strategy
	int mini_state;
	
} Strategy;

void init_strategy(CtrlStruct *cvs);
void main_strategy(CtrlStruct *cvs);
void round_trip(CtrlStruct *cvs, int mini_state);
void full_speed(CtrlStruct *cvs, int mini_state);
void obstacles_avoidance(CtrlStruct *cvs);

#endif

