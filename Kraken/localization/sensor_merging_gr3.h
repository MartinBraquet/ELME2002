
#ifndef _SENSOR_MERGING_H_
#define _SENSOR_MERGING_H_

#include "../CtrlStruct_gr3.h"

#define LIM_VAR_X		0.001
#define LIM_VAR_Y		0.001
#define LIM_VAR_THETA	0.015

#define LIM_ERR_X_COMB		0.04
#define LIM_ERR_Y_COMB		0.04
#define LIM_ERR_THETA_COMB	0.05

#define SENSOR_RECOMBINATION_NEEDED 2
#define SENSOR_UPDATE_NEEDED 1
#define SENSOR_UPDATE_NOT_NEEDED 0

int checkPositionUpdateNeeded(CtrlStruct *cvs);
void mergeSensorData(CtrlStruct *cvs);
void recombineSensorData(CtrlStruct *cvs);
void Kalman_filter(CtrlStruct *cvs);


#endif
