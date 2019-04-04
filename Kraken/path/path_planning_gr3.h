/*! 
 * \file path_planning_gr3.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_EX_H_
#define _PATH_PLANNING_EX_H_

#include "../CtrlStruct_gr3.h"

#define NB_TARGETS 9 // Number of targets
#define NB_POINTS_OBSTACLES 560 // Number of obstacles
#define NUMBER_ELEM_WALL (NB_POINTS_OBSTACLES - 3)
#define NB_BEACONS 5 // Number of targets

#define MAP_STEP 0.02 // Distance between wall obstacles

// Map specific positions (corners, bars,...)
#define MAP_X1 1.0
#define MAP_X2 0.563
//#define MAP_X2 0.563
#define MAP_X3 -0.97
#define MAP_X4 0.38
#define MAP_Y1 1.5
#define MAP_Y2 1.05
#define MAP_Y3 1.0
#define MAP_Y4 0.0

#define LOW_SPEED 1.0
#define MEDIUM_SPEED 2.0
#define HIGH_SPEED 1.0

#define LOW_SENSIBILITY_ROTATION 0.5
#define MEDIUM_SENSIBILITY_ROTATION 2.0
#define HIGH_SENSIBILITY_ROTATION 3.0

#define DROP_SPEED_ROTATION 5.0
#define DROP_SPEED_LINEAR 15.0

enum {DROP_TARGET_STATE_1, DROP_TARGET_STATE_2, DROP_TARGET_STATE_3, DROP_TARGET_STATE_4};

/// path-planning main structure
struct PathPlanning
{
	double targets[2][NB_TARGETS]; // (x, y) coordinates in lines 0 and 1, and targets in each column
	double speed_target[NB_TARGETS]; // speed to reach target
	double omega_sensibility[NB_TARGETS]; // rotates more or less compare to the linear speed
	int    index_next_target;
	double obstacles[2][NB_POINTS_OBSTACLES]; // (x, y) coordinates in lines 0 and 1, and obstacles in each column
	double obstacles_rho_0[NB_POINTS_OBSTACLES];
	
	double target_basis_team[2];
	
	double k_att;
	double k_rep[NB_POINTS_OBSTACLES];
	double k_opp;
	double opp_rho_0;
	
	double beacons[2][NB_BEACONS]; // (x, y) beacons coordinates
	
	double v_l; // Imposed left wheel speed to reach target
	double v_r; // Imposed right wheel speed to reach target
	
	double t_flag;
	int next;    // Flag to go to the next target
	int drop_flag;    // FSM for the dropping motion
	int drop_action;    // Drop action
};

void path_planning_update(CtrlStruct *cvs);
void init_path_planning(CtrlStruct *cvs);
void drop_target(CtrlStruct *cvs);


#endif
