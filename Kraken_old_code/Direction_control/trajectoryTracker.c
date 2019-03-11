#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <cmath>
#include "trajectoryTracker.h"


//Method call to create the controller
void init_trajectoryTracker(TrajectoryTracker* target,PositionMapping* currentLocation,PIcontroller* angularPI,PIcontroller* linearPI)
{	
	target->currentLocation = currentLocation;
	target->xTarget =0.0;
	target->yTarget =0.0;
	
	target->angularPositionPI  = angularPI;
	target->linearPositionPI = linearPI;

	target->angleRef = 0.0;
	target->angleMes = 0.0;
	target->distanceRef = 0.0; // always 0 
	target->distanceMes = 0.0;
}

void run_trajectoryTracker(TrajectoryTracker* target, double* omega_ref)
{	
	update_target(target);
	computeSpeed(target,omega_ref);
}

void computeSpeed(TrajectoryTracker* target,double* omega_ref)
{
	double omegaRef_lin = 0.0; 
	double omegaRef_ang = 0.0; 

	if(target->angleRef-target->angleMes < 15) // VERIFIER SI C EST BIEN EN DEGREE
	omegaRef_lin = run_controller(target->linearPositionPI,target->distanceRef,target->distanceMes);
	
	omegaRef_ang =  run_controller(target->angularPositionPI,target->angleRef,target->angleMes);
	
	omega_ref[0] = omegaRef_lin + omegaRef_ang;
	omega_ref[1] = omegaRef_lin - omegaRef_ang;
	
}

//Method call to fix a taget
void set_target(TrajectoryTracker* target,double x,double y) 
{
	target->xTarget = x;
	target->yTarget = y;
}

//Methode use to update the value of reference
void update_target(TrajectoryTracker* target)
{
	
	double deltaX = target->xTarget-target->currentLocation->x ;
	double deltaY = target->yTarget-target->currentLocation->y ;
	
	//When the robot needs to reach a target then the distance reference is 0		
	target->distanceMes = sqrt(deltaX*deltaX+deltaY*deltaY);
	target->angleMes = target->currentLocation-> theta; //rad
			
		
	//The robot angle is count positive when y is greater than 0 and negative otherwise. It goes from -PI/2 to PI/2
	//It depends on which quaters the robot wants to moove	

	double alpha = abs_value(atan(deltaY/deltaX));
			
	if(deltaY > 0)  //In this the angle of the robot must be >0
	{
		if(deltaX > 0)
		{
			target->angleRef = alpha;  //The minus M_PI is to fit with the robot coordinates	
		}
		else if(deltaX < 0)
		{
			target->angleRef = M_PI-alpha; 
		}
		else
		{
			target->angleRef =  M_PI/2; 
		}
		
	}
	else if(deltaY < 0)
	{
		if(deltaX > 0)
		{
			target->angleRef = -alpha; 
		}
		else if(deltaX < 0)//In this case the robot must angle between 0 and pi/2
		{
			target->angleRef = alpha-M_PI; 
		}
		else
		{
			target->angleRef =  -M_PI/2; 
		}
			
	}
	if(deltaY == 0) 
	{
		if(deltaX >= -0.1)
		{
			target->angleRef = 0;
		}
		else
		{
			target->angleRef =  -M_PI; 
		}
	}

	if(target->distanceMes < 0.05)
	{
		target->distanceMes = 0.0;
		target->angleRef = target->angleMes;
	}

}

void print_tracker(TrajectoryTracker* target)
{
	printf("L'angle mesuré vaut : %lf \n",target->angleMes);
	printf("L'angle de réf vaut : %lf \n",target->angleRef);
	printf("dist    mesuré vaut : %lf \n",target->distanceMes);
	printf("dist    de réf vaut : %lf \n",target->distanceRef);

}

double abs_value(double number)
{
	if(number<0)
		return -number;
	else
		return number;
}
