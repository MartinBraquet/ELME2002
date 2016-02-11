#include <cstdio>
#include <stdio.h>
#include <cstdio>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>


typedef struct PositionMapping
{	
	//robot properties
	double distance_wheels;
	double wheels_radius;
	

	//odometer postion
	double angle_R; //rad
	double angle_L;
	double old_angle_R; //rad
	double old_angle_L;
	double delta_angle_R;
	double delta_angle_L;
	int tour;

	//rotation radius
	double radiusRobot; //rad

	
	double theta; //rad
	double x; //m
	double y; //m
	
	double dxdt; //m/s
	double dthetadt;  //rad/s

	double correctionLin;
	double correctionRot;

}PositionMapping;


//way to initialize the position memory structure
void initMapping(PositionMapping* positionOnMap,double wheels_radius,double distance_wheels);

//function to call in order to localise the minibot on the map
void run_mapping(PositionMapping* positionOnMap,double speed_Left,double speed_Right);

//function to print the position
void print_mapping(PositionMapping* positionOnMap);

//function to update the coordinate
void update_coordinates(PositionMapping* positionOnMap);

//update the distance driven by each wheels
void update_MappingLP(PositionMapping* positionOnMapdouble, double angle_wheel_L,double angle_wheel_R);

//update the angular postion on the robot
void compute_dthetadt(PositionMapping* positionOnMap);

//update the linear speed off the robot
void compute_radius(PositionMapping* positionOnMap);

//update the linear speed off the robot
void compute_dxdt(PositionMapping* positionOnMap);

//transform  the wheels encoder in rad
double count_to_rad(int count);

//reset function
void reset_all(PositionMapping* positionOnMap);
void reset_x(PositionMapping* positionOnMap);
void reset_y(PositionMapping* positionOnMap);
void reset_theta(PositionMapping* positionOnMap);

//function to set a location
void set_XYtheta(PositionMapping* positionOnMap,double x, double y,double theta);
