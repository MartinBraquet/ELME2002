/*! 
 * \file speed_controller_gr3.h
 * \brief Control of the motors
 */
#ifndef _SPEED_CONTROLLER_GR3_H_
#define _SPEED_CONTROLLER_GR3_H_

#include "../CtrlStruct_gr3.h"


typedef struct Motor{

	// Controller
	double Ki;
	double Kp;
	
	double K; ///< gain of the power electronics
	
	double komega;  ///< omega gain of the sensor
	
	double integral_error;
	
} Motor;

typedef struct MotorStruct{

    Motor *l_motor;
    Motor *r_motor;
    
    double Un; ///< V
    double In; ///< A
    double Nn; ///< tr/min

    double Ra; ///< omh
    double kphi;  ///< kphi of the motor
    double red; ///< reduction ratio
    
} MotorStruct;

void init_speed_controller(CtrlStruct* cvs);
void free_speed_controller(CtrlStruct* cvs);
void run_speed_controller(CtrlStruct *cvs, double l_sp_ref, double r_sp_ref);
int saturation(double *x, double xsat);
double compute_angle_wheel_motor(int count);
double compute_speed_wheel_motor(int speed);
double compute_angle_wheel_odo(int count);
double compute_speed_wheel_odo(int speed);


#endif //ifndef
