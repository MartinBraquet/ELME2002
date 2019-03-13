/*! 
 * \file speed_controller_gr3.c
 * \brief Control of the motors
 */

#include <stdio.h>
#include "speed_controller_gr3.h"
#include "speed_regulation_gr3.h"


// Angle of the wheel motors in rad
double compute_angle_wheel_motor(int count){
	return (count * 360 / 37750) * M_PI / 180;
}

// Rotation speed of the motor wheels in rad/s: compute each 65536 cycles and the CLK is at 50 MHz
double compute_speed_wheel_motor(int speed){
	return (speed * 2 * M_PI / 37750 / 65536.0 * 50e6);
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

double run_motor(CtrlStruct *cvs, Motor *theMotor, double omega_ref, double wheel_speed, double dt){
    double motor_speed = wheel_speed / theMotor->komega * cvs->motor_str->red;

	double delta_omega = omega_ref * cvs->motor_str->red - motor_speed;
	//printf("omega_ref, wheel_speed, delta omega: %f %f %f \n", omega_ref, wheel_speed, delta_omega);

	double ua_p_ref = delta_omega * theMotor->Kp + theMotor->Ki * (dt * delta_omega + theMotor->integral_error);

	// Current limitation
	int current_sat = saturation(&ua_p_ref, cvs->motor_str->Ra * cvs->motor_str->In);

	double ua_ref = ua_p_ref + cvs->motor_str->kphi * motor_speed;
	

	// Voltage limitation
	int voltage_sat = saturation(&ua_ref, cvs->motor_str->Un);

	if (!current_sat && !voltage_sat) {
		//printf("saturation");
	    theMotor->integral_error = dt * delta_omega + theMotor->integral_error;
	}

	double ua = (double) ua_ref / theMotor->K;
	
	//printf("tension Ua : %lf, tension Ua_ref %lf \n", ua, ua_ref); 

	return ua;
}

void run_speed_controller(CtrlStruct *cvs, double l_sp_ref, double r_sp_ref){
	
	// printf("Run speed controller\n");
	double dt = cvs->inputs->t - cvs->sp_reg->last_t;
	double Ua_left_out  = run_motor(cvs, cvs->motor_str->l_motor, l_sp_ref, cvs->inputs->motor_enc_l_wheel_speed, dt);
	double Ua_right_out = run_motor(cvs, cvs->motor_str->r_motor, r_sp_ref, cvs->inputs->motor_enc_r_wheel_speed, dt);
	
	cvs->outputs->wheel_commands[L_ID] = Ua_left_out * 100.0 / cvs->motor_str->Un;
	cvs->outputs->wheel_commands[R_ID] = Ua_right_out * 100.0 / cvs->motor_str->Un;
	
	 //printf("Ua left%f \n",Ua_left_out);
	 //printf("Ua right%f \n",Ua_right_out);
	return;
}

void init_speed_controller(CtrlStruct *cvs){

	cvs->motor_str->Un = 24.0;
	cvs->motor_str->In = 0.82;
	cvs->motor_str->Nn = 4770.0;
	cvs->motor_str->Ra = 5.84;
	cvs->motor_str->kphi = 0.03783;
	cvs->motor_str->red = 19.0;

    cvs->motor_str->l_motor = (Motor*) malloc(sizeof(struct Motor));
    cvs->motor_str->r_motor = (Motor*) malloc(sizeof(struct Motor));
	
	cvs->motor_str->l_motor->Ki = 0.3367;
	cvs->motor_str->l_motor->Kp = 0.0415;
	cvs->motor_str->l_motor->K = 1.0;
	cvs->motor_str->l_motor->komega = 0.9998;
	cvs->motor_str->l_motor->integral_error = 0.0;

	cvs->motor_str->r_motor->Ki = cvs->motor_str->l_motor->Ki;
	cvs->motor_str->r_motor->Kp = cvs->motor_str->l_motor->Kp;
	cvs->motor_str->r_motor->K = 1.0;
	cvs->motor_str->r_motor->komega = 0.9998;
	cvs->motor_str->r_motor->integral_error = 0.0;

	return;
}

void free_speed_controller(CtrlStruct *cvs){

    free(cvs->motor_str->l_motor);
    free(cvs->motor_str->r_motor);

	return;
}

