#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <cmath>
#include "mappingPosition.h"

typedef struct TrajectoryTracker
{
	PositionMapping* currentLocation;

	double xTarget;
	double yTarget;

	double angleRef;
 	double angleMes;
	double distanceRef;
	double distanceMes; 

}TrajectoryTracker;

void init_trajectoryTracker(TrajectoryTracker* target,PositionMapping* currentLocation);
void run_trajectoryTracker(TrajectoryTracker* target);
void set_target(TrajectoryTracker* target,double x, double y);
void update_target(TrajectoryTracker* target);
void print_tracker(TrajectoryTracker* target);
double abs_value(double number);


