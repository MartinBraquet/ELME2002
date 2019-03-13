/*! 
 * \file triangulation.h
 * \brief beacon detection
 */

#ifndef _TRIANGULATION_GR3_H_
#define _TRIANGULATION_GR3_H_ 

#include "../CtrlStruct_gr3.h"

#define BEACONS_SQUARE 0.2 

// function prototype
void triangulation_mean_data(CtrlStruct *cvs);
void triangulation_angles(CtrlStruct *cvs);
void triangulation_distances_two_nearest(CtrlStruct *cvs);
void triangulation_distances_all_three(CtrlStruct *cvs);

#endif
