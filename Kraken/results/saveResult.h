//#ifndef _SAVE_RESULTS_H_
//#define _SAVE_RESULTS_H_

#include <stdlib.h>
#include <stdio.h>

typedef struct SaveFileStruct
{
	FILE* fichierSpeedWheel;
	FILE* fichierPostionRobot;
		
}SaveFileStruct;

void saveResults_init(SaveFileStruct *save);
void saveResults_finish(SaveFileStruct *save);

//#endif
