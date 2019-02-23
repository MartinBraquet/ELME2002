#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <cmath>
#include "mappingPosition.h"
#include "direction_controller.h"

/*
 * Contains all the information to guide the follow to a pecise point
 */
typedef struct TrajectoryTracker
{
	PositionMapping* currentLocation;
	
	PIcontroller* angularPositionPI;
	PIcontroller* linearPositionPI;

	double xTarget;
	double yTarget;

	double angleRef;
 	double angleMes;
	double distanceRef;
	double distanceMes; 

}TrajectoryTracker;

/*
 * List of all the point where the robot should go trough
 */
 
 /*
typedef struct TrajectoryPath
{
	double* xList;
	double* yList;

	int numberOfPoint;
 	int isReached;
 	
}TrajectoryPath;
*/

//Initialisation method
void init_trajectoryTracker(TrajectoryTracker* target,PositionMapping* currentLocation,PIcontroller* angularPI, PIcontroller* linearPI);
//void init_trajectoryPath(TrajectoryPath* path, int n, double* x, double* y);

// Methods for trajectoryTracker
void run_trajectoryTracker(TrajectoryTracker* target,double* omega_ref);
void set_target(TrajectoryTracker* target,double x, double y);
void update_target(TrajectoryTracker* target);
void print_tracker(TrajectoryTracker* target);
double abs_value(double number);
void computeSpeed(TrajectoryTracker* target,double* omega_ref);

/*
// Methods for trajectoryPath 
void add_step(TrajectoryPath* path, double x, double y, int position);
void remove_step(TrajectoryPath* path, int n);
void clear(TrajectoryPath* path);
void target_reached(TrajectoryPath* path,PositionMapping* currentLocation, double precision);
*/
