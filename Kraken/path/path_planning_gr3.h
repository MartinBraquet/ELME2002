/*! 
 * \file path_planning_gr3.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_EX_H_
#define _PATH_PLANNING_EX_H_

#include "../CtrlStruct_gr3.h"


#define NB_TARGETS 11 // Number of targets
#define NB_POINTS_OBSTACLES 108 // Number of obstacles

/// path-planning main structure
struct PathPlanning
{
	double targets[2][NB_TARGETS]; // (x, y) coordinates in lines 0 and 1, and targets in each column
	int    index_next_target;
	double obstacles[2][NB_POINTS_OBSTACLES]; // (x, y) coordinates in lines 0 and 1, and obstacles in each column
	double obstacles_rho_0[NB_POINTS_OBSTACLES];
	
	double target_basis_team[2];
	
	double k_att;
	double k_rep[NB_POINTS_OBSTACLES];
	
	double beacons[2][3]; // (x, y) beacons coordinates
	
	double v_l; // Imposed left wheel speed to reach target
	double v_r; // Imposed right wheel speed to reach target
	double speed_coeff;
	
	double t_flag;
	double next;    // Flag to go to the next target
};

void path_planning_update(CtrlStruct *cvs);
void init_path_planning(CtrlStruct *cvs);


#endif
