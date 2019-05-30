//#ifdef _SAVE_RESULTS_H_

#include <stdlib.h>
#include "saveResult.h"

void saveResults_init(SaveFileStruct *save)
{
    save->fichierSpeedWheel = NULL;
    save->fichierPostionRobot = NULL;

    save->fichierSpeedWheel = fopen("/home/pi/Desktop/ELME2002/Kraken/results/motorRegulation_test.txt", "w+");
    save->fichierPostionRobot = fopen("/home/pi/Desktop/ELME2002/Kraken/results/poistion_test.txt", "w+");
    
    fprintf(save->fichierSpeedWheel, "time, vitesse_ref_L, vitesse_L, vitesse_ref_R, vitesse_R \n ");
    fprintf(save->fichierPostionRobot,"time, position_x, position_y, position_theta, target_x, target_y \n");
    
    //fprintf(cvs->save->fichierSpeedWheel, "%0.5f, %0.5f , %0.5f, %0.5f, %0.5f\n",cvs->inputs->t,cvs->path->v_r,cvs->inputs->motor_enc_l_wheel_speed,cvs->path->v_r,cvs->inputs->motor_enc_r_wheel_speed);
	//fprintf(cvs->save->fichierPostionRobot, "%0.5f, %0.5f , %0.5f, %0.5f \n",cvs->inputs->t,cvs->rob_pos->x,cvs->rob_pos->y,cvs->rob_pos->theta);
     
}

void saveResults_finish(SaveFileStruct *save)
{
    fclose(save->fichierSpeedWheel);
    fclose(save->fichierPostionRobot);
}

//#endif
