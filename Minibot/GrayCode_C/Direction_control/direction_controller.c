#include <cstdio>
#include <stdio.h>
#include <cstdio>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <cmath>
#include "direction_controller.h"


void initPIController(PIcontroller* controller,double kp,double ki)
{
	controller->kp = kp; 
	controller->ki = ki;

	controller->error = 0.0; 
	controller->errorSum = 0.0; 
}


double run_positionRotation_controller(PIcontroller* controller,double angleRef,double angleMes,double dt)
{
	//error
	controller->error = (angleRef - angleMes)/180*M_PI;
	controller->errorSum = controller->errorSum +  controller->error*dt;
	
	return controller->kp*controller->error  + controller->ki*controller->errorSum; 
}

double run_positionForward_controller(PIcontroller* controller,double distanceRef,double distanceMes,double dt)
{

	//error
	controller->error = distanceRef - distanceMes;
	controller->errorSum = controller->errorSum +  controller->error*dt;
	
	return controller->kp*controller->error  + controller->ki*controller->errorSum; 
}


double compute_distance(double deltaAngle,int mode)
{	
	double width = 0.0252;

	if(mode == 1) // When approching a target
		width = 0.015;
	
	
	if(deltaAngle == 0)
	{
		return -1;
	}
	else
	{
		return (width/tan(deltaAngle/180*M_PI));
	}
	
}



