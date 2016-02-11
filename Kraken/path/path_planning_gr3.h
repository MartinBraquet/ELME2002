/*! 
 * \author Nicolas Van der Noot
 * \file path_planning_gr3.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_EX_H_
#define _PATH_PLANNING_EX_H_

#include "../CtrlStruct_gr3.h"

#if ROBOTICS_COURSE
    #include "namespace_ctrl.h"
    NAMESPACE_INIT(ctrlGr3);
#endif

#define NB_POINTS_OBSTACLES 141 // Number of obstacles

/// path-planning main structure
struct PathPlanning
{
	double targets_x[11];
	double targets_y[11];
	double obstacles_x[NB_POINTS_OBSTACLES];
	double obstacles_y[NB_POINTS_OBSTACLES];
	double obstacles_rho_0[NB_POINTS_OBSTACLES];
	double obstacles_weight[NB_POINTS_OBSTACLES];
};

void path_planning_update(CtrlStruct *cvs);
void init_path_planning(CtrlStruct *cvs);

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif

#endif
