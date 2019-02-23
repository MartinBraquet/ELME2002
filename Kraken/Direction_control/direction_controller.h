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
    double dt;
    double min;
    double max;
    double error; 
    double errorSum;
 
}PIcontroller;


//Way to initialize the controller
void initPIController(PIcontroller* controller, double kp, double ki,double dt,double min, double max);

//Proportionnal controler for a angula direction, return a reference speed
double run_controller(PIcontroller* controller,double Ref,double Mes);







