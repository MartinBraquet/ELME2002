#include <cstdio>
#include <stdio.h>
#include <cstdio>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <cmath>
#include "direction_controller.h"


void initPIController(PIcontroller* controller,double kp,double ki,double dt,double min, double max)
{	
	controller->kp = kp; 
	controller->ki = ki;
	
	controller->dt = dt;
	
	controller->min = min;
	controller->max = max;  

	controller->error = 0.0; 
	controller->errorSum = 0.0; 
}


double run_controller(PIcontroller* controller,double Ref,double Mes)
{
	//error
	controller->error = (Ref-Mes);
	controller->errorSum = controller->errorSum +  controller->error*controller->dt;
	double sum = controller->kp*controller->error  + controller->ki*controller->errorSum; 
	
	// Limit the linear speed 
	if(sum > controller->max)
		sum = controller->max; 
	
	if(sum < controller->min)
		sum = controller->min;
		
	return sum;
}
	




