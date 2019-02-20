#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include "CtrlStruct.h"
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "../IO/COM/SPI/SPI.hh"
#include "../IO/COM/CAN/CAN.hh"
#include "../IO/COM/SPI/Specific/SPI_CAN.hh"

double BuffertoAngle(char buffer);
void init_speed_controller(CtrlStruct* theCtrlStruct);
void run_speed_controller(CtrlStruct* theCtrlStruct, double *omega_ref);
double run_motor(CtrlStruct* theCtrlStruct, Motor* theMotor, double omega_ref, double wheel_speed);
int saturation(double *x, double xsat);
double compute_angle_wheel_motor(int count);

// Angle of the wheel motors in degree
double compute_angle_wheel_motor(int count){
	return (count * 360 / 27610) % 360;
}

// Rotation speed of the motor wheels in rad/s: compute each 65536 cycles and the CLK is at 50 MHz
double compute_speed_wheel_motor(int speed){
	return (speed * 2 * M_PI / 27610.0 / 65536.0 * 50e6);
}

void run_speed_controller(CtrlStruct* theCtrlStruct, double *omega_ref){
	
	double Ua_left_out = run_motor(theCtrlStruct,theCtrlStruct->theUserStruct->theMotor_l,omega_ref[L_ID],theCtrlStruct->theCtrlIn->l_wheel_speed);
	double Ua_right_out = run_motor(theCtrlStruct,theCtrlStruct->theUserStruct->theMotor_r,omega_ref[R_ID],theCtrlStruct->theCtrlIn->r_wheel_speed);
	
	theCtrlStruct->theCtrlOut->wheel_commands[L_ID] = Ua_left_out * 100.0 / theCtrlStruct->theUserStruct->Un;
	theCtrlStruct->theCtrlOut->wheel_commands[R_ID] = Ua_right_out * 100.0 / theCtrlStruct->theUserStruct->Un;
	
	 //printf("%f \n",Ua_left_out);
	 //printf("%f \n",Ua_right_out);
	return;
}

void init_speed_controller(CtrlStruct* theCtrlStruct){

	theCtrlStruct->theUserStruct->Un = 24.0;
	theCtrlStruct->theUserStruct->In = 0.78;
	theCtrlStruct->theUserStruct->Nn = 6470.0;
	theCtrlStruct->theUserStruct->Ra = 7.1;
	theCtrlStruct->theUserStruct->R = 0.03;
	theCtrlStruct->theUserStruct->kphi = 0.0261;
	theCtrlStruct->theUserStruct->dt = 0.001;
	theCtrlStruct->theUserStruct->red = 14.0;

    	theCtrlStruct->theUserStruct->theMotor_l = (Motor*) malloc(sizeof(struct Motor));
    	theCtrlStruct->theUserStruct->theMotor_r = (Motor*) malloc(sizeof(struct Motor));
	
	theCtrlStruct->theUserStruct->theMotor_l->Ki = 0.1194;
	theCtrlStruct->theUserStruct->theMotor_l->Kp = 0.0391;
	theCtrlStruct->theUserStruct->theMotor_l->K = 1.0;
	theCtrlStruct->theUserStruct->theMotor_l->komega = 0.9998;
	theCtrlStruct->theUserStruct->theMotor_l->integral_error = 0.0;

	theCtrlStruct->theUserStruct->theMotor_r->Ki = 0.1194;
	theCtrlStruct->theUserStruct->theMotor_r->Kp = 0.0391;
	theCtrlStruct->theUserStruct->theMotor_r->K = 1.0;
	theCtrlStruct->theUserStruct->theMotor_r->komega = 0.9998;
	theCtrlStruct->theUserStruct->theMotor_r->integral_error = 0.0;

	return;
}

double run_motor(CtrlStruct* theCtrlStruct, Motor* theMotor, double omega_ref, double wheel_speed){
    double motor_speed = wheel_speed / theMotor->komega * theCtrlStruct->theUserStruct->red;

	double delta_omega = omega_ref * theCtrlStruct->theUserStruct->red - motor_speed;
	//printf("omega_ref, wheel_speed, delta omega: %f %f %f \n", omega_ref, wheel_speed, delta_omega);

	double ua_p_ref = delta_omega * theMotor->Kp
	                    + theMotor->Ki * (theCtrlStruct->theUserStruct->dt * delta_omega + theMotor->integral_error);

	// Current limitation
	int current_sat = saturation(&ua_p_ref, theCtrlStruct->theUserStruct->Ra * theCtrlStruct->theUserStruct->In);

	double ua_ref = ua_p_ref + theCtrlStruct->theUserStruct->kphi * motor_speed;
	

	// Voltage limitation
	int voltage_sat = saturation(&ua_ref, theCtrlStruct->theUserStruct->Un);

	if (!current_sat && !voltage_sat) {
	    theMotor->integral_error = theCtrlStruct->theUserStruct->dt * delta_omega + theMotor->integral_error;
	}

	double ua = (double) ua_ref / theMotor->K;

	return ua;
}

int saturation(double *x, double xsat){
    if (*x > xsat) {
	    *x = xsat;
	    return 1;
	}
	else if (*x < -xsat ) {
	    *x = -xsat;
	    return 1;
	}
	else {
	    return 0;
	}
}


