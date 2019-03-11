// Make use of the CMake build system or compile manually, e.g. with:
// gcc -std=c99 example.c -lsweep

#include <sweep/sweep.h>
#include "../CtrlStruct_gr3.h"

void die(sweep_error_s error);
void init_LIDAR(CtrlStruct *cvs);
void get_LIDAR_data(CtrlStruct *cvs);
void free_LIDAR(CtrlStruct *cvs);
