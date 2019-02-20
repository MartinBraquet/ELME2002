#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <cmath>
#include "mappingPosition.h"


//way to initialize the position memory structure
void initMapping(PositionMapping* positionOnMap,double wheels_radius,double distance_wheels)
{	
	positionOnMap->distance_wheels  = distance_wheels;
	positionOnMap->wheels_radius  =  wheels_radius;
		
	positionOnMap->angle_R  = 0.0;
	positionOnMap->angle_L  = 0.0;
	positionOnMap->old_angle_R  = 0.0;
	positionOnMap->old_angle_L  = 0.0;
	positionOnMap->delta_angle_R  = 0.0;
	positionOnMap->delta_angle_L  = 0.0;
	positionOnMap->tour = 0;
	
	positionOnMap->radiusRobot = 0.0;

	positionOnMap->theta = 0.0;
	positionOnMap->x  = 0.0;	
	positionOnMap->y  = 0.0;	
	positionOnMap->dxdt = 0.0;
	positionOnMap->dthetadt = 0.0;

	positionOnMap->correctionLin = 1;
	positionOnMap->correctionRot = 1;
}


void run_mapping(PositionMapping* positionOnMap,double omega_wheel_L,double omega_wheel_R)
{
	//First we update the data
	 update_MappingLP(positionOnMap,omega_wheel_L,omega_wheel_R);
	//Then we compute the radius and the speed
	 compute_radius(positionOnMap);
	 compute_dxdt(positionOnMap);
	//Then we compute the angle variation
	 compute_dthetadt(positionOnMap);
	//An finally we can update the coordinate
	 update_coordinates(positionOnMap);
}

void print_mapping(PositionMapping* positionOnMap)
{
	printf("Position en x : %lf \n",positionOnMap->x);
	printf("Position en y : %lf \n",positionOnMap->y);
	printf("Direction theta: %lf \n",positionOnMap->theta);

	printf("Vitesse linéaire : %lf \n",positionOnMap->dxdt);
	printf("Vitesse de rotation: %lf\n",positionOnMap->dthetadt);
}


//update the distance driven by each wheels
void update_MappingLP(PositionMapping* positionOnMap,double angle_wheel_L,double angle_wheel_R)
{ 
	

	positionOnMap->old_angle_R = positionOnMap->angle_R; //rad
	positionOnMap->old_angle_L = positionOnMap->angle_L;
	positionOnMap->angle_R = angle_wheel_R; //rad
	positionOnMap->angle_L = angle_wheel_L; //rad

	positionOnMap->delta_angle_R = count_to_rad((int)(positionOnMap->angle_R - positionOnMap->old_angle_R));
	positionOnMap->delta_angle_L = count_to_rad((int)(positionOnMap->angle_L - positionOnMap->old_angle_L));
}


void update_coordinates(PositionMapping* positionOnMap)
{
	positionOnMap->theta = (positionOnMap->theta + positionOnMap->dthetadt);

	positionOnMap->x  = positionOnMap->x + positionOnMap->dxdt*cos(positionOnMap->theta);
	positionOnMap->y  = positionOnMap->y + positionOnMap->dxdt*sin(positionOnMap->theta); 	

	if(positionOnMap->theta>M_PI) 
	{
		positionOnMap->theta = positionOnMap->theta - 2*M_PI;
		positionOnMap->tour = positionOnMap->tour+1;
	}
	else if(positionOnMap->theta<-M_PI) 
	{
		positionOnMap->theta = positionOnMap->theta + 2*M_PI;
		positionOnMap->tour = positionOnMap->tour-1;
	}
	
}

void compute_dthetadt(PositionMapping* positionOnMap)
{
	positionOnMap->dthetadt =  positionOnMap->correctionRot*positionOnMap->wheels_radius*(positionOnMap->delta_angle_L-positionOnMap->delta_angle_R)/(positionOnMap->distance_wheels);
}

//Not usefull for the odometrie
void compute_radius(PositionMapping* positionOnMap)
{
	positionOnMap->radiusRobot = 0.0;
}

void compute_dxdt(PositionMapping* positionOnMap)
{
	positionOnMap->dxdt = positionOnMap->correctionLin*positionOnMap->wheels_radius*(positionOnMap->delta_angle_R+positionOnMap->delta_angle_L)/2;
}

double count_to_rad(int count)
{
	return count *2*M_PI/ 27610.0;
}

//function to set a location
void set_XYtheta(PositionMapping* positionOnMap,double x, double y,double theta)
{
	positionOnMap->x = x;
	positionOnMap->y = y;
	positionOnMap->theta = theta;
}

//function to reset the location
void reset_all(PositionMapping* positionOnMap)
{
	reset_x(positionOnMap);
	reset_y(positionOnMap);
	reset_theta(positionOnMap);
}
void reset_x(PositionMapping* positionOnMap)
{
	positionOnMap->x = 0;
}
void reset_y(PositionMapping* positionOnMap)
{
	positionOnMap->y = 0;
}
void reset_theta(PositionMapping* positionOnMap)
{
	positionOnMap->theta = 0;
}


