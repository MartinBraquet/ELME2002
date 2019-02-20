#include <cstdio>
#include <stdio.h>
#include <cstdio>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>


typedef struct PIcontroller
{
    double kp;
    double ki;
    
    double error; 
    double errorSum;
 
}PIcontroller;



//Proportionnal controler for a angula direction, return a reference speed
double run_positionRotation_controller(PIcontroller* controller,double angleRef,double angleMes,double dt);

//Proportionnal controler linear direction, return a reference speed
double run_positionForward_controller(PIcontroller* controller,double distanceRef,double distanceMes,double dt);

//Compute a distance base on the beacon width
double compute_distance(double angle,int mode);


//Way to initialize the controller
void initPIController(PIcontroller* controller, double kp, double ki);






