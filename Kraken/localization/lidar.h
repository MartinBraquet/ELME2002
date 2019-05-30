
// Make use of the CMake build system or compile manually, e.g. with:
// gcc -std=c99 example.c -lsweep

#ifndef _LIDAR_H_
#define _LIDAR_H_

#include "../CtrlStruct_gr3.h"
#include "../path/path_planning_gr3.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#include <unistd.h>


typedef struct LIDAR_data
{
	double lidar_mean_angles[NB_BEACONS];
	double lidar_mean_distances[NB_BEACONS];
	
	double relative_theta_opp;
	int opp_detected;
	double dist_opp;
	int stop;
	
	int nearest_indexes[NB_BEACONS];
	
} LIDAR_data;

void init_LIDAR(CtrlStruct *cvs);
void get_LIDAR_data(CtrlStruct *cvs);
void free_LIDAR(CtrlStruct *cvs);

#endif
