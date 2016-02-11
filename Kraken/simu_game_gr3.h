/*! 
 * \file simu_game_gr3.h
 * \brief choose between controller for simulation game or for the real robot
 */

#ifndef _SIMU_GAME_EX_H_
#define _SIMU_GAME_EX_H_

#if ROBOTICS_COURSE
    #include "namespace_ctrl.h"
    NAMESPACE_INIT(ctrlGr3);
#endif

#ifdef SIMU_PROJECT
	#define SIMU_GAME // comment this line to see in simulation the controller which will be tested on the real robot
#endif

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif

#endif
