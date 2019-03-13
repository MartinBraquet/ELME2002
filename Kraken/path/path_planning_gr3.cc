#include "path_planning_gr3.h"
#include "../localization/init_pos_gr3.h"
#include "../localization/opp_pos_gr3.h"


/*! \brief initialize the path-planning algorithm
 * 
 * \param[in,out] cvs controller main structure
 */ 
void init_path_planning(CtrlStruct *cvs)
{
    int i, j, k;
    
    PathPlanning *path = cvs->path;
    
    path->index_next_target = 0;
    path->drop_flag = DROP_TARGET_STATE_1;
    path->k_att = 10.0;
    path->k_opp = 4.0;
    path->opp_rho_0 = 0.6;
    double rho_0_wall = 0.25;
    double rho_0_bar = 0.2;
    double k_rep_wall = 0.01;
    double k_rep_bar = 0.02;
    path->t_flag = 0.0;
    path->next = 0;
    path->drop_action = 0;
    
    if (cvs->robot_team == TEAM_YELLOW) {
	path->beacons[0][0] = -0.95; path->beacons[1][0] = -1.595;
	path->beacons[0][1] =  0.95; path->beacons[1][1] = -1.595;
	path->beacons[0][2] =  0.8 ; path->beacons[1][2] =  0.0;
	path->beacons[0][3] =  0.0 ; path->beacons[1][3] =  1.595;
	path->beacons[0][4] = -0.9 ; path->beacons[1][4] =  0.0;
    } else {
	path->beacons[0][0] = -0.0 ; path->beacons[1][0] = -1.595;
	path->beacons[0][1] =  0.8 ; path->beacons[1][1] =  0.0;
	path->beacons[0][2] =  0.95; path->beacons[1][2] =  1.595;
	path->beacons[0][3] = -0.95; path->beacons[1][3] =  1.595;
	path->beacons[0][4] = -0.9 ; path->beacons[1][4] =  0.0;
    }
	
    double target_basis_team[2] = {0.5, cvs->plus_or_minus * 1.1};
    for (i=0; i<2; i++) {
	path->target_basis_team[i] = target_basis_team[i];
    }
    
    double targets[2][NB_TARGETS] = {{0.0, 0.3, 0.0, 0.4, -0.8, target_basis_team[0],
                                       0.25, 0.4, target_basis_team[0]},
                                     {1.0, 0.3, 0.0, cvs->plus_or_minus * 0.15,
                                      -cvs->plus_or_minus * 0.8, target_basis_team[1],
                                      -cvs->plus_or_minus * 0.25, -cvs->plus_or_minus * 0.15, target_basis_team[1]}};
                                     
    double speed_target[NB_TARGETS] = {HIGH_SPEED, MEDIUM_SPEED, MEDIUM_SPEED, MEDIUM_SPEED, MEDIUM_SPEED, MEDIUM_SPEED,
                                       MEDIUM_SPEED, MEDIUM_SPEED, MEDIUM_SPEED};
                                       
    double omega_sensibility[NB_TARGETS] = {HIGH_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, HIGH_SENSIBILITY_ROTATION,  
                                            HIGH_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION,
                                            MEDIUM_SENSIBILITY_ROTATION, HIGH_SENSIBILITY_ROTATION, MEDIUM_SENSIBILITY_ROTATION};
    
	for (j=0; j<NB_TARGETS; j++) {
	    for (i=0; i<2; i++) {
		    path->targets[i][j] = targets[i][j];
		}
		path->speed_target[j] = speed_target[j];
		path->omega_sensibility[j] = omega_sensibility[j];
		//printf("Targets x: %f   ; Targets x: %f\n", cvs->path->targets_x[i], cvs->path->targets_y[i]);
	}
	
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
        path->obstacles_rho_0[j+k] = 0.2;
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
	
}

void drop_target(CtrlStruct *cvs) {

    RobotPosition *rob_pos = cvs->rob_pos;
    PathPlanning *path = cvs->path;
    
    // finite state machine (FSM)
	switch (path->drop_flag)
	{
		case DROP_TARGET_STATE_1:
		
			//printf("State: DROP_TARGET_STATE_1\n");
		    
	        path->v_l =   DROP_SPEED_ROTATION * ((rob_pos->theta > 0) - (rob_pos->theta < 0));
	        path->v_r = - DROP_SPEED_ROTATION * ((rob_pos->theta > 0) - (rob_pos->theta < 0));

            if (fabs(rob_pos->theta) < M_PI/50.0)
			{
			    path->drop_flag = DROP_TARGET_STATE_2;
			}
			break;

		case DROP_TARGET_STATE_2:
		
			//printf("State: DROP_TARGET_STATE_2\n");
			
	        path->v_l = DROP_SPEED_LINEAR;
	        path->v_r = DROP_SPEED_LINEAR;
		    
            if (rob_pos->x > 0.75) {
                path->drop_flag = DROP_TARGET_STATE_3;
            }
			break;

		case DROP_TARGET_STATE_3:
		
			//printf("State: DROP_TARGET_STATE_3\n");
			
	        path->v_l = -DROP_SPEED_LINEAR;
	        path->v_r = -DROP_SPEED_LINEAR;
		    
            if (rob_pos->x < 0.4) {
                path->drop_flag = DROP_TARGET_STATE_4;
            }
			break;
			
		case DROP_TARGET_STATE_4:
		
			//printf("State: DROP_TARGET_STATE_4\n");
		    
	        path->v_l =  cvs->plus_or_minus * DROP_SPEED_ROTATION;
	        path->v_r = -cvs->plus_or_minus * DROP_SPEED_ROTATION;

            if (fabs(rob_pos->theta + cvs->plus_or_minus * 3.0 / 4.0 * M_PI) < M_PI/50.0 && path->index_next_target != NB_TARGETS)
			{
			    path->index_next_target++;
			    path->drop_flag = DROP_TARGET_STATE_1;
			    path->drop_action = 0;
			}
			break;
	
		default:
			printf("Error: unknown drop target state : %d !\n", path->drop_flag);
			exit(EXIT_FAILURE);
	}

}

/*! \brief update the path-planning algorithm
 * 
 * \param[in,out] cvs controller main structure
 */
void path_planning_update(CtrlStruct *cvs)
{
    int i, j;
    double k, rho;
    PathPlanning *path = cvs->path;
    RobotPosition *rob_pos = cvs->rob_pos;
    OpponentsPosition *opp_pos = cvs->opp_pos;
    int index_next_target = path->index_next_target;
    
    double target[2] = {path->targets[0][index_next_target], path->targets[1][index_next_target]}; // Next target in (x, y)
    
    double rob_pos_2D[2] = {rob_pos->x, rob_pos->y};
    
    
    printf("TARGET: (%1.3f, %1.3f)\n", target[0], target[1]);
    
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
	       // printf("Obs %d  ", j);
	        k = path->k_rep[j] * (1.0 / rho - 1.0 / path->obstacles_rho_0[j]) / pow(rho, 3);
	        for (i=0; i<2; i++) {
                F[i] += k * (rob_pos_2D[i] - path->obstacles[i][j]);
            }
	    }
	}

        // Opponents repulsive forces
	for (j=0; j < opp_pos->nb_opp; j++) {
	    rho = sqrt(pow(rob_pos_2D[0] - opp_pos->x[j], 2) + pow(rob_pos_2D[1] - opp_pos->y[j], 2));
	    printf("Opp %f %f \n", opp_pos->x[j], opp_pos->y[j]);
	    if (rho < path->opp_rho_0) {
	        k = path->k_opp * (1.0 / rho - 1.0 / path->opp_rho_0) / pow(rho, 3);
            F[0] += k * (rob_pos_2D[0] - opp_pos->x[j]);
            F[1] += k * (rob_pos_2D[1] - opp_pos->y[j]);
	    }
	}
	
	
	double alpha = atan2(F[1], F[0]) - rob_pos->theta;
	double norm_F = path->speed_target[index_next_target] * sqrt(F[0]*F[0] + F[1]*F[1]);
	double V = norm_F * cos(alpha);
	double omega = path->omega_sensibility[index_next_target] * norm_F * sin(alpha);
	
	path->v_l = V - cvs->robot_dimensions->wheel_axle * omega;
	path->v_r = V + cvs->robot_dimensions->wheel_axle * omega;
	
	//printf("V = (%1.2f, %1.2f)\n", path->v_l, path->v_r);
	
	// printf("F = (%1.2f, %1.2f)   V: %1.2f    omega: %1.2f\n", F[0], F[1], V, omega);
	
	// printf("Time delay: %f\n", cvs->inputs->t - path->t_flag);
	
	//printf("Next = %d\n", path->next);
	
	if (pow(rob_pos_2D[0] - path->targets[0][index_next_target], 2) + pow(rob_pos_2D[1] - path->targets[1][index_next_target], 2) < 0.01) {	    
		//path->index_next_target++;
		//printf("Next = 0\n");
	}

}
