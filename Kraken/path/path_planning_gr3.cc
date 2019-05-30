 #include "path_planning_gr3.h"
#include "../localization/init_pos_gr3.h"
#include "../localization/opp_pos_gr3.h"
#include "../regulation/speed_regulation_gr3.h"
#include "../IO/client.h"
#include <semaphore.h>
#include <fcntl.h>


/*! \brief initialize the path-planning algorithm
 * 
 * \param[in,out] cvs controller main structure
 */ 
void init_path_planning(CtrlStruct *cvs)
{
    int i, j, k;
    
    PathPlanning *path = cvs->path;
    
    path->index_next_target = 0;
    path->k_att = 10.0;
    path->k_opp = 0.5;
    path->opp_rho_0 = 0.6;
    double rho_0_wall = 0.2;
    double rho_0_bar = 0.2;
    double k_rep_wall = 0.0;
    double k_rep_bar = 0.0;
    path->t_flag = 0.0;
    path->next = 0;
    path->state_path = 0;
    path->action_flag = 0;
    
    if (cvs->robot_team == TEAM_YELLOW) {
	path->beacons[0][0] = -0.93; path->beacons[1][0] = -1.56;
	path->beacons[0][1] =  0.93; path->beacons[1][1] = -1.56;
	path->beacons[0][2] =  0.8 ; path->beacons[1][2] =  0.0;
	path->beacons[0][3] =  0.0 ; path->beacons[1][3] =  1.55;
	path->beacons[0][4] = -1.1 ; path->beacons[1][4] =  0.0;
    } else {
	path->beacons[0][0] =  0.0 ; path->beacons[1][0] = -1.55;
	path->beacons[0][1] =  0.8 ; path->beacons[1][1] =  0.0;
	path->beacons[0][2] =  0.93; path->beacons[1][2] =  1.56;
	path->beacons[0][3] = -0.93; path->beacons[1][3] =  1.56;
	path->beacons[0][4] = -1.1 ; path->beacons[1][4] =  0.0;
    }
	
	
    /*double targets[2][NB_TARGETS] = {{-0.1,0.1, 0.2, -0.5, -0.5, 0.0, -0.2, 0.25, target_basis_team[0]},
				     {cvs->plus_or_minus*0.1, -0.6, -0.0, -0.2, -1.3, target_basis_team[1],  -cvs->plus_or_minus * 0.15, target_basis_team[1], 0.0}};
     */	
     
     //until target 4 - make a turn around the chaos zone to bring back the palet to the red rectangle
     //	     target 5 - go near the rack to launch action 1 to take all the palet
     //      target 6 - go near the acceletor to launch action 2 and 3
     
     /*
     // Base program with accel
    double targets[2][NB_TARGETS+1] = {{0.2, -0.1, 0.2, 0.1, -0.4, 0.2, 0.2 ,-0.72, -0.7, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    				     {cvs->plus_or_minus*0.7, -cvs->plus_or_minus*0.1,-cvs->plus_or_minus*0.1,-cvs->plus_or_minus*0.7,-cvs->plus_or_minus*1.2, -cvs->plus_or_minus*0.8, -cvs->plus_or_minus*0.5, -cvs->plus_or_minus*0.1, cvs->plus_or_minus*0.75, cvs->plus_or_minus*0.2, 0.0, 0.0, 0.0, 0.0, 0.0,0.0}};
       
    double orientation_target[NB_TARGETS+1] = {10,10, -3*cvs->plus_or_minus*M_PI/8,10.0, 10.0, M_PI, M_PI, -(M_PI+0.3)/2, 0.0, M_PI, 10.0, 10.0, 10.0, 10.0, 10.0};
    
    int action_target[NB_TARGETS+1] = {10,10, 10, 10,10, 1, 2, 10, 8, 9, 10, 10, 10, 10, 10, 10};
    
    double time_positioning_target[NB_TARGETS+1] = {0.0,0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    */
    
    if (cvs->main_strategy == 0) {
	    
	double targets[2][NB_TARGETS] = {{-0.2, 0.15, 0.0, -0.4, 0.2, 0.1 ,-0.35, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  				         {-cvs->plus_or_minus*0.1,-cvs->plus_or_minus*0.12,-cvs->plus_or_minus*0.9,-cvs->plus_or_minus*1.2, -cvs->plus_or_minus*0.87, -cvs->plus_or_minus*0.55, -cvs->plus_or_minus*1.1, -cvs->plus_or_minus*1.1, cvs->plus_or_minus*0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

	
	double orientation_target[NB_TARGETS] = {-cvs->plus_or_minus*M_PI/2 + M_PI/2, -3*cvs->plus_or_minus*M_PI/8,10.0, 10.0, M_PI, M_PI, cvs->plus_or_minus*M_PI/2, 0.0, M_PI, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
	    
	int action_target[NB_TARGETS] = {10, 10, 10,10, 1, 2, 9, 10, 10, 10, 10, 10, 10, 10, 10};
	
	double time_positioning_target[NB_TARGETS] = {0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double x_positioning_target[NB_TARGETS] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
	double y_positioning_target[NB_TARGETS] = {10.0, 10.0, 10.0, 10.0, -cvs->plus_or_minus*0.88, -cvs->plus_or_minus*0.58, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
	
	double precision_target[NB_TARGETS] = {0.1, 0.15, 0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
	    
	double speed_target[NB_TARGETS] = {HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, MEDIUM_SPEED, HIGH_SPEED, MEDIUM_SPEED, MEDIUM_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED};

	double omega_sensibility[NB_TARGETS] = {HIGH_SENSIBILITY_ROTATION, HIGH_SENSIBILITY_ROTATION,  
						    HIGH_SENSIBILITY_ROTATION,  
						    HIGH_SENSIBILITY_ROTATION, 
						    HIGH_SENSIBILITY_ROTATION,
						    MEDIUM_SENSIBILITY_ROTATION,
						    MEDIUM_SENSIBILITY_ROTATION,
						    MEDIUM_SENSIBILITY_ROTATION,
						    MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION};
	    
	int score_target[NB_TARGETS] = {0, 0, 0, 18, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
	for (j=0; j<NB_TARGETS; j++) {
	    for (i=0; i<2; i++) {
		    path->targets[i][j] = targets[i][j];
		}
		path->speed_target[j] = speed_target[j];
		path->omega_sensibility[j] = omega_sensibility[j];
		path->orientation_target[j] = orientation_target[j];
		path->action_target[j] = action_target[j];
		path->time_positioning_target[j] = time_positioning_target[j];
		path->x_positioning_target[j] = x_positioning_target[j];
		path->y_positioning_target[j] = y_positioning_target[j];
		path->score_target[j] = score_target[j];
		path->precision_target[j] = precision_target[j] * precision_target[j];
		printf("Targets x: %f   ; Targets y: %f\n", path->targets[0][j], path->targets[1][j]);
	}
	
	
	printf("End init path\n");
	
    } else if (cvs->main_strategy == 1) {
	    
	// target 0: go to accelerator
	// target 1: execute actions in the accelerator
	// target 2: position to push atoms on floor
	// target 3: intermediate position while pushing atoms on the floor
	// target 4: final position for pushing atoms on the floor
	double targets[2][NB_TARGETS] = {{-0.77, -0.77, 0.15, 0.1, -0.4, 0.2, 0.1 ,-0.35, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    				     {0.0, 0.0, -cvs->plus_or_minus*0.08,-cvs->plus_or_minus*0.7,-cvs->plus_or_minus*1.2, -cvs->plus_or_minus*0.87, -cvs->plus_or_minus*0.55, -cvs->plus_or_minus*1.1, -cvs->plus_or_minus*1.1, cvs->plus_or_minus*0.0, 0.0, 0.0, 0.0, 0.0}};
       
	double orientation_target[NB_TARGETS] = {10.0, -(cvs->plus_or_minus*M_PI-0.3)/2.0,-cvs->plus_or_minus*M_PI/2, -3*cvs->plus_or_minus*M_PI/8,10.0, M_PI, M_PI, cvs->plus_or_minus*M_PI/2, 0.0, M_PI, 10.0, 10.0, 10.0, 10.0};
	    
	int action_target[NB_TARGETS] = {5,10,6, 10,10, 1, 2, 9, 10, 10, 10, 10, 10, 10, 10};
	    
	double time_positioning_target[NB_TARGETS] = {0.0, 4.0,0.0, 0.0, 0.0, 4.0, 4.0, 0.0, 0.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0};
	double x_positioning_target[NB_TARGETS] = {10.0, -0.85, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
	double y_positioning_target[NB_TARGETS] = {10.0, 10.0, 10.0, 10.0, 10.0, -cvs->plus_or_minus*0.88, -cvs->plus_or_minus*0.58, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
	
	double precision_target[NB_TARGETS] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
	    
	double speed_target[NB_TARGETS] = {HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, MEDIUM_SPEED, HIGH_SPEED, MEDIUM_SPEED, MEDIUM_SPEED, HIGH_SPEED};

	double omega_sensibility[NB_TARGETS] = {HIGH_SENSIBILITY_ROTATION, HIGH_SENSIBILITY_ROTATION, HIGH_SENSIBILITY_ROTATION,  
						    HIGH_SENSIBILITY_ROTATION,  
						    HIGH_SENSIBILITY_ROTATION, 
						    HIGH_SENSIBILITY_ROTATION,
						    MEDIUM_SENSIBILITY_ROTATION,
						    MEDIUM_SENSIBILITY_ROTATION,
						    MEDIUM_SENSIBILITY_ROTATION,
						    MEDIUM_SENSIBILITY_ROTATION};
	    
	int score_target[NB_TARGETS] = {0, 0, 0, 0, 18, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0};
    
    	for (j=0; j<NB_TARGETS; j++) {
	    for (i=0; i<2; i++) {
		    path->targets[i][j] = targets[i][j];
		}
		path->speed_target[j] = speed_target[j];
		path->omega_sensibility[j] = omega_sensibility[j];
		path->orientation_target[j] = orientation_target[j];
		path->action_target[j] = action_target[j];
		path->time_positioning_target[j] = time_positioning_target[j];
		path->score_target[j] = score_target[j];
		path->precision_target[j] = precision_target[j] * precision_target[j];
		printf("Targets x: %f   ; Targets y: %f\n", path->targets[0][j], path->targets[1][j]);
	}
	
    } else if (cvs->main_strategy == 2) {
	    
	double targets[2][NB_TARGETS] = {{0.0, 0.0, 0.15, 0.0, -0.4, 0.2, 0.1 ,-0.35, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  				         {cvs->plus_or_minus*0.6, cvs->plus_or_minus*0.6,-cvs->plus_or_minus*0.12,-cvs->plus_or_minus*0.8,-cvs->plus_or_minus*1.2, -cvs->plus_or_minus*0.88, -cvs->plus_or_minus*0.55, -cvs->plus_or_minus*1.1, -cvs->plus_or_minus*1.1, cvs->plus_or_minus*0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

	
	double orientation_target[NB_TARGETS] = {-cvs->plus_or_minus*M_PI/2, -cvs->plus_or_minus*M_PI/2, -3*cvs->plus_or_minus*M_PI/8,10.0, 10.0, M_PI, M_PI, cvs->plus_or_minus*M_PI/2, 0.0, M_PI, 10.0, 10.0, 10.0, 10.0, 10.0};
	    
	int action_target[NB_TARGETS] = {10, 10, 10, 10,10, 1, 2, 9, 10, 10, 10, 10, 10, 10, 10};
	
	double time_positioning_target[NB_TARGETS] = {0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 0.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0};
	double x_positioning_target[NB_TARGETS] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
	double y_positioning_target[NB_TARGETS] = {10.0,10.0, 10.0, 10.0, 10.0, -cvs->plus_or_minus*0.88, -cvs->plus_or_minus*0.58, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
	
	double precision_target[NB_TARGETS] = {0.1, 0.1, 0.15, 0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
	    
	double speed_target[NB_TARGETS] = {HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, MEDIUM_SPEED, HIGH_SPEED, MEDIUM_SPEED, MEDIUM_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED};

	double omega_sensibility[NB_TARGETS] = {HIGH_SENSIBILITY_ROTATION, HIGH_SENSIBILITY_ROTATION, HIGH_SENSIBILITY_ROTATION,  
						    HIGH_SENSIBILITY_ROTATION,  
						    HIGH_SENSIBILITY_ROTATION, 
						    HIGH_SENSIBILITY_ROTATION,
						    MEDIUM_SENSIBILITY_ROTATION,
						    MEDIUM_SENSIBILITY_ROTATION,
						    MEDIUM_SENSIBILITY_ROTATION,
						    MEDIUM_SENSIBILITY_ROTATION};
	    
	int score_target[NB_TARGETS] = {0, 0, 0, 18, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
	for (j=0; j<NB_TARGETS; j++) {
	    for (i=0; i<2; i++) {
		    path->targets[i][j] = targets[i][j];
		}
		path->speed_target[j] = speed_target[j];
		path->omega_sensibility[j] = omega_sensibility[j];
		path->orientation_target[j] = orientation_target[j];
		path->action_target[j] = action_target[j];
		path->time_positioning_target[j] = time_positioning_target[j];
		path->x_positioning_target[j] = x_positioning_target[j];
		path->y_positioning_target[j] = y_positioning_target[j];
		path->score_target[j] = score_target[j];
		path->precision_target[j] = precision_target[j] * precision_target[j];
		printf("Targets x: %f   ; Targets y: %f\n", path->targets[0][j], path->targets[1][j]);
	}
	
	
	printf("End init path\n");
	
    } 
  
	/*double targets[2][NB_TARGETS] = {{-0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2},
			     {0.5, -1.0, 0.5, -1.0, 0.5, -1.0, 0.5, -1.0, 0.5, -1.0, 0.5, -1.0, 0.5, -1.0, 0.5}};
	double orientation_target[NB_TARGETS] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
	int action_target[NB_TARGETS] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
	double time_positioning_target[NB_TARGETS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double speed_target[NB_TARGETS] = {LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED};
	double omega_sensibility[NB_TARGETS] = {MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION};
      */
    

	
    //to change the inital start of the robot (debuging purposes) 
    //cvs->rob_pos->x = -0.1;
    //cvs->rob_pos->y = -cvs->plus_or_minus*0.9;
	
    /*******************************************************
    ***                 Obstacles (walls + bar)          ***
    *******************************************************/
	
	for (j=0; j < NB_POINTS_OBSTACLES; j++)
	{
	    if (j < NUMBER_ELEM_WALL) {
            path->obstacles_rho_0[j] = rho_0_wall;
            path->k_rep[j] = k_rep_wall;
	    } else {
            path->obstacles_rho_0[j] = rho_0_bar;
            path->k_rep[j] = k_rep_bar;
	    }
	    //printf("%f %f %f %f\n",cvs->path->obstacles[0][j],cvs->path->obstacles[1][j],cvs->path->obstacles_rho_0[j],cvs->path->k_rep[j]);
	}
	
	j = 0;
	
	// Bottom wall
	for(k=0; -MAP_X1 + k * MAP_STEP < MAP_X1; k++) {
        path->obstacles[0][j+k] = -MAP_X1 + k * MAP_STEP;
        path->obstacles[1][j+k] = -MAP_Y1;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	// Right wall
	for(k=0; -MAP_Y1 + k * MAP_STEP < -MAP_Y2; k++) {
        path->obstacles[0][j+k] = MAP_X1;
        path->obstacles[1][j+k] = -MAP_Y1 + k * MAP_STEP;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	for(k=0; MAP_X1 - k * MAP_STEP > MAP_X2; k++) {
        path->obstacles[0][j+k] = MAP_X1 - k * MAP_STEP;
        path->obstacles[1][j+k] = -MAP_Y2;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	for(k=0; -MAP_Y2 + k * MAP_STEP < MAP_Y2; k++) {
        path->obstacles[0][j+k] = MAP_X2;
        path->obstacles[1][j+k] = -MAP_Y2 + k * MAP_STEP;
        path->obstacles_rho_0[j+k] = 0.2;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	for(k=0; MAP_X2 + k * MAP_STEP < MAP_X1; k++) {
        path->obstacles[0][j+k] = MAP_X2 + k * MAP_STEP;
        path->obstacles[1][j+k] = MAP_Y2;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	for(k=0; MAP_Y2 + k * MAP_STEP < MAP_Y1; k++) {
        path->obstacles[0][j+k] = MAP_X1;
        path->obstacles[1][j+k] = MAP_Y2 + k * MAP_STEP;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	// Top wall
	for(k=0; MAP_X1 - k * MAP_STEP > -MAP_X1; k++) {
        path->obstacles[0][j+k] = MAP_X1 - k * MAP_STEP;
        path->obstacles[1][j+k] = MAP_Y1;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	// Left wall
	for(k=0; MAP_Y1 - k * MAP_STEP > MAP_Y3; k++) {
        path->obstacles[0][j+k] = -MAP_X1;
        path->obstacles[1][j+k] = MAP_Y1 - k * MAP_STEP;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	for(k=0; -MAP_X1 + k * MAP_STEP < MAP_X3; k++) {
        path->obstacles[0][j+k] = -MAP_X1 + k * MAP_STEP;
        path->obstacles[1][j+k] = MAP_Y3;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	for(k=0; MAP_Y3 - k * MAP_STEP > -MAP_Y3; k++) {
        path->obstacles[0][j+k] = MAP_X3;
        path->obstacles[1][j+k] = MAP_Y3 - k * MAP_STEP;
        //path->obstacles_rho_0[j+k] = 0.2;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	for(k=0; MAP_X3 - k * MAP_STEP > -MAP_X1; k++) {
        path->obstacles[0][j+k] = MAP_X3 - k * MAP_STEP;
        path->obstacles[1][j+k] = -MAP_Y3;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	for(k=0; -MAP_Y3 - k * MAP_STEP > -MAP_Y1; k++) {
        path->obstacles[0][j+k] = -MAP_X1;
        path->obstacles[1][j+k] = -MAP_Y3 - k * MAP_STEP;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	// Bar near accelerator
	for(k=0; MAP_X2 - MAP_STEP - k * MAP_STEP > MAP_X4; k++) {
        path->obstacles[0][j+k] = MAP_X2 - MAP_STEP - k * MAP_STEP;
        path->obstacles[1][j+k] = MAP_Y4;
        //printf("%d %f %f\n",j+k, path->obstacles[0][j+k],path->obstacles[1][j+k]);
	}
	j += k;
	
	//printf("\n\n\n\n%d\n\n\n\n\n\n\n\n", j);
	
}

void action_target_1(CtrlStruct *cvs) {
	sem_t *actions_semaphore;
	actions_semaphore = sem_open("/actions_semaphore", O_CREAT);
	
	//cvs->rob_pos->x = 0.38;
	//cvs->rob_pos->theta = M_PI;
	
	printf("\n\n\n\naction 1\n\n\n\n");
		
	cvs->outputs->next_action = 1;
	sem_post(actions_semaphore);
}

void action_target_2(CtrlStruct *cvs) {
	sem_t *actions_semaphore;
	actions_semaphore = sem_open("/actions_semaphore", O_CREAT);
	
	//cvs->rob_pos->x = 0.38;
	//cvs->rob_pos->theta = M_PI;
	
	printf("\n\n\n\naction 2\n\n\n\n");
		
	cvs->outputs->next_action = 2;
	sem_post(actions_semaphore);
}

void action_target_3(CtrlStruct *cvs) {
	sem_t *actions_semaphore;
	actions_semaphore = sem_open("/actions_semaphore", O_CREAT);
		
	printf("\n\n\n\naction 3\n\n\n\n");
		
	cvs->outputs->next_action = 3;
	sem_post(actions_semaphore);
}

void action_target_4(CtrlStruct *cvs) {
	sem_t *actions_semaphore;
	actions_semaphore = sem_open("/actions_semaphore", O_CREAT);
	
	printf("\n\n\n\naction 4\n\n\n\n");
		
	cvs->outputs->next_action = 4;
	sem_post(actions_semaphore);
	
}

void action_target_5(CtrlStruct *cvs) {
	sem_t *actions_semaphore;
	actions_semaphore = sem_open("/actions_semaphore", O_CREAT);
	
	printf("\n\n\n\naction 5\n\n\n\n");
		
	cvs->outputs->next_action = 5;
	sem_post(actions_semaphore);
	
}

void action_target_6(CtrlStruct *cvs) {
	sem_t *actions_semaphore;
	actions_semaphore = sem_open("/actions_semaphore", O_CREAT);
	
	printf("\n\n\n\naction 6\n\n\n\n");
		
	cvs->outputs->next_action = 6;
	sem_post(actions_semaphore);
	
}

void action_target_9(CtrlStruct *cvs) {
	sem_t *actions_semaphore;
	actions_semaphore = sem_open("/actions_semaphore", O_CREAT);
	
	printf("\n\n\n\naction 9\n\n\n\n");
		
	cvs->outputs->next_action = 9;
	sem_post(actions_semaphore);
	
}


void go_to_target(CtrlStruct *cvs) {

    int i, j;
    double k, rho;
    PathPlanning *path = cvs->path;
    RobotPosition *rob_pos = cvs->rob_pos;
    OpponentsPosition *opp_pos = cvs->opp_pos;
    int index_next_target = path->index_next_target;

    //printf("%d\n", index_next_target);
    
    double target[2] = {path->targets[0][index_next_target], path->targets[1][index_next_target]}; // Next target in (x, y)
    
    
    printf("TARGET %d: (%1.3f, %1.3f) %f %d\n", index_next_target,target[0], target[1], path->orientation_target[index_next_target], path->action_target[index_next_target]);
    
    double rob_pos_2D[2] = {rob_pos->x, rob_pos->y};    
    
    double F[2];
    
    // Attractive force
    for (i=0; i<2; i++) {
	F[i] = - path->k_att * (rob_pos_2D[i] - target[i]);
	}
	
	//printf("F_att = (%1.2f, %1.2f)   ", F[0], F[1]);
	
	// Repulsive forces
	for (j=0; j < NB_POINTS_OBSTACLES; j++) {
	    rho = sqrt(pow(rob_pos_2D[0] - path->obstacles[0][j], 2) + pow(rob_pos_2D[1] - path->obstacles[1][j], 2));
	    if (rho < path->obstacles_rho_0[j]) {
		//printf("Obs %d  (%f, %f)\n", j, path->obstacles[0][j], path->obstacles[1][j]);
		//printf("Me %d  (%f, %f)\n", j, rob_pos_2D[0], rob_pos_2D[1]);
		//printf("Rho max_rho %f %f\n", pow(rob_pos_2D[0] - path->obstacles[0][j], 2), pow(rob_pos_2D[1] - path->obstacles[1][j], 2));
		k = path->k_rep[j] * (1.0 / rho - 1.0 / path->obstacles_rho_0[j]) / pow(rho, 3);
		for (i=0; i<2; i++) {
		F[i] += k * (rob_pos_2D[i] - path->obstacles[i][j]);
	    }
	    }
	}

	// Opponents repulsive forces
	for (j=0; j < opp_pos->nb_opp; j++) {
	    rho = sqrt(pow(rob_pos_2D[0] - opp_pos->x[j], 2) + pow(rob_pos_2D[1] - opp_pos->y[j], 2));
	    //printf("Opp (%f, %f) \n", opp_pos->x[j], opp_pos->y[j]);
	    if (rho < path->opp_rho_0) {
		k = path->k_opp * (1.0 / rho - 1.0 / path->opp_rho_0) / pow(rho, 3);
		F[0] += k * (rob_pos_2D[0] - opp_pos->x[j]);
		F[1] += k * (rob_pos_2D[1] - opp_pos->y[j]);
	    }
	}
	
	
	double alpha = atan2(F[1], F[0]) - rob_pos->theta;
	double norm_F = path->speed_target[index_next_target] * sqrt(F[0]*F[0] + F[1]*F[1]);
	/*
	if (norm_F < 1.0) {
		norm_F = 3 * norm_F;
	}
	*/
	double V = norm_F * cos(alpha);
	double omega = path->omega_sensibility[index_next_target] * norm_F * sin(alpha);
	
	path->v_l = V - cvs->robot_dimensions->wheel_axle * omega;
	path->v_r = V + cvs->robot_dimensions->wheel_axle * omega;
	
	//printf("V = (%1.2f, %1.2f)\n", path->v_l, path->v_r);
	
	// printf("F = (%1.2f, %1.2f)   V: %1.2f    omega: %1.2f\n", F[0], F[1], V, omega);
	
	// printf("Time delay: %f\n", cvs->inputs->t - path->t_flag);
	
	//printf("Next = %d\n", path->next);
	

	if(index_next_target >= NB_TARGETS){
		path->v_l = 0.0; path->v_r = 0.0;
	}

	
	// once we arrive in the target
	if ((pow(rob_pos_2D[0] - path->targets[0][index_next_target], 2) + pow(rob_pos_2D[1] - path->targets[1][index_next_target], 2) < path->precision_target[index_next_target])) {	   
		path->state_path++;
		path->v_l = 0.0; path->v_r = 0.0;
	}
	
	/*
	if (inputs->t - path->t_flag_target > 30.0) {
		path->index_next_target++;
	}
	*/
	
}


void orientation_of_robot(CtrlStruct *cvs) {
	
	PathPlanning *path = cvs->path;
	RobotPosition *rob_pos = cvs->rob_pos;
	
	double orientation_target = path->orientation_target[path->index_next_target];
	
	if (path->orientation_target[path->index_next_target] > M_PI) {
		path->state_path++;
		path->v_l = 0.0; path->v_r = 0.0;
	} else {
		if (orientation_target > rob_pos->theta) {
			path->v_l = -ROTATION_SPEED; path->v_r = ROTATION_SPEED;
		} else {
			path->v_l = ROTATION_SPEED; path->v_r = -ROTATION_SPEED;
		}
		if (abs(path->orientation_target[path->index_next_target] - rob_pos->theta) < M_PI/20) {
			path->state_path++;
			path->t_flag = cvs->inputs->t;
			path->v_l = 0.0; path->v_r = 0.0;
		}
	}
}

void positioning_of_robot(CtrlStruct *cvs){
	PathPlanning *path = cvs->path;
	RobotPosition *rob_pos = cvs->rob_pos;
	CtrlIn *inputs = cvs->inputs;
	
	int index_next_target = path->index_next_target;
	
	if (inputs->t - path->t_flag >= path->time_positioning_target[index_next_target]) {
		path->state_path++;
		path->v_l = 0.0; path->v_r = 0.0;
	} else {
		double omega = SPEED_TOKEN * (path->orientation_target[index_next_target] - rob_pos->theta) / 50.0;
		double delta_v = 0;
		if (path->y_positioning_target[index_next_target] < 2.0) {
			delta_v = SPEED_TOKEN * (path->targets[1][index_next_target] - rob_pos->y) * 2;
		} else if (path->x_positioning_target[index_next_target] < 2.0) {
			delta_v = SPEED_TOKEN * (path->targets[0][index_next_target] - rob_pos->x) * 2;
		}
		printf("omega: %f delta_vy: %f\n", omega, delta_v);
		path->v_l = -SPEED_TOKEN + omega - delta_v; path->v_r = -SPEED_TOKEN - omega + delta_v;
	}	
	
}

void action_robot(CtrlStruct *cvs) {
	
	PathPlanning *path = cvs->path;
	RobotPosition *rob_pos = cvs->rob_pos;
	
	int index_next_target = path->index_next_target;
	
	if (path->action_target[index_next_target] == 10) {
		path->index_next_target++;
		path->state_path = 0;
		path->v_l = 0.0; path->v_r = 0.0;
		return;
	}
	
	printf("ACTION\n");
	static int action_FSM_state;
	
	switch (action_FSM_state){
		case 0:
			switch (path->action_target[index_next_target]){
				case 1:
					action_target_1(cvs);
				break;
				case 2:
					action_target_2(cvs);
				break;
				case 3:
					action_target_3(cvs);
				break;
				case 4:
					action_target_4(cvs);
				break;
				case 5:
					action_target_5(cvs);
				break;
				case 6:
					action_target_6(cvs);
				break;
				case 9:
					action_target_9(cvs);
				break;
				default:
					printf("No such action: %d\n", path->action_target[index_next_target]);
				break;
				
			}
			//path->t_flag_target = cvs->inputs->t;
			path->v_l = 0.0; path->v_r = 0.0;
			action_FSM_state++;
			break;
		// action finished
		case 1:
			if (cvs->outputs->action_in_progress == 0){
				path->index_next_target++;
				path->state_path = 0;
				action_FSM_state = 0;
				cvs->outputs->score += path->score_target[index_next_target];
			}
			else{
				path->v_l = 0.0; path->v_r = 0.0;
			}
			break;
		default:
			printf("ON TARGET FSM ERROR: no such state found\n");
	}
	
}



/*! \brief update the path-planning algorithm
 * 
 * \param[in,out] cvs controller main structure
 */
void path_planning_update(CtrlStruct *cvs)
{    
    printf("target index: %d\n", cvs->path->index_next_target);
    printf("state_path: %d\n", cvs->path->state_path);
    switch (cvs->path->state_path) {
	    case 0:
		go_to_target(cvs);
		break;
	    case 1:
		orientation_of_robot(cvs);
		break;
	    case 2:
		positioning_of_robot(cvs);
		break;
	    case 3:
		action_robot(cvs);
		break;
	    default: break;
	}
}
