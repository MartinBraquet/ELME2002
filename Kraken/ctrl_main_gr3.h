/*! 
 * \file ctrl_main_gr3.h
 * \brief main header of the controller
 */

#ifndef _CTRL_MAIN_GR3_H_
#define _CTRL_MAIN_GR3_H_

#include "CtrlStruct_gr3.h"
#include <stdlib.h>

void controller_init(CtrlStruct *cvs);
void controller_loop(CtrlStruct *cvs);
void controller_finish(CtrlStruct *cvs);

#endif
