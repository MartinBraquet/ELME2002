#include "path_planning_gr3.h"
#include "../localization/init_pos_gr3.h"

#if ROBOTICS_COURSE
    NAMESPACE_INIT(ctrlGr3);
#endif

/*! \brief initialize the path-planning algorithm
 * 
 * \param[in,out] cvs controller main structure
 */
void init_path_planning(CtrlStruct *cvs)
{
    int i, j;
    
    PathPlanning *path = cvs->path;
    
    path->index_next_target = 0;
    path->k_att = 6.0;	
    double rho_0_wall = 0.1;
    double rho_0_bar = 0.05;
    int number_elem_wall = 100;
	double k_rep = 5.0;
    path->t_flag = 0.0;
    path->next = 0;
    path->speed_coeff = 2.0;
    
    double beacons[2][3] = {{0.0, 1.062, -1.062},
                            {-1.562, 1.562, 1.562}};
	for (j=0; j<3; j++) {
	    for (i=0; i<2; i++) {	
		    path->beacons[i][j] = beacons[i][j];
		}
	}
    
    double targets[2][NB_TARGETS] = {{-0.0, 0.45, 0.45, 0.4, 0.4, -0.8, -0.8, 0.0},
                                     {0.0, 0.15, -0.15, 1.3, -1.3, 0.8, -0.8, 0.0}};
	for (j=0; j<NB_TARGETS; j++) {	
	    for (i=0; i<2; i++) {
		    path->targets[i][j] = targets[i][j];
		}
		//printf("Targets x: %f   ; Targets x: %f\n", cvs->path->targets_x[i], cvs->path->targets_y[i]);
	}
	
	double target_basis_team[2] = {0.8, cvs->plus_or_minus * 1.25};
	for (i=0; i<2; i++) {
	    path->target_basis_team[i] = target_basis_team[i];
	}
	
    //Obstacles
	double obstacles[2][NB_POINTS_OBSTACLES] = {
	// X coodinates
	{-1060, -900, -800, -700, -600, -500, -400, -300, -200, -100, 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1060,   // Bottom wall
	  1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060,               // Right wall
	 -1060, -900, -800, -700, -600, -500, -400, -300, -200, -100, 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1060,     // Top wall
	 -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060,  // Left wall
	 340, 360, 380, 400, 420, 440, 460, 480
	},
	// Y coordinates
    {-1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560,  // Bottom wall
    -1400, -1300, -1200, -1100, -1000, -900, -800, -700, -600, -500, -400, -300, -200, -100, 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400,   // Right wall
	 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, // Top wall
	 -1400, -1300, -1200, -1100, -1000, -900, -800, -700, -600, -500, -400, -300, -200, -100, 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400,  // Left wall
	 0, 0, 0, 0, 0, 0, 0, 0
	 }};

	for (j=0; j<NB_POINTS_OBSTACLES; j++)
	{
	    if (j < number_elem_wall) {
            path->obstacles_rho_0[j] = rho_0_wall;
	    } else {
            path->obstacles_rho_0[j] = rho_0_bar;
	    }
		path->k_rep[j] = k_rep;
	    for (i=0; i<2; i++) {
		    path->obstacles[i][j] = 0.001 * obstacles[i][j];
		    //printf("Obstacle x: %f   ; Obstacle x: %f   ; Obstacle rho_0: %f   ; Obstacle weight: %f\n",
		    //    cvs->path->obstacles_x[i], cvs->path->obstacles_y[i], cvs->path->obstacles_rho_0[i], cvs->path->obstacles_weight[i]);
	    }
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
    int index_next_target = path->index_next_target;
    
    double target[2] = {path->targets[0][index_next_target], path->targets[1][index_next_target]}; // Next target in (x, y)
    
    //printf("TARGET: (%1.3f, %1.3f)\n", target[0], target[1]);
    
    double rob_pos_2D[2] = {rob_pos->x, rob_pos->y};
    
    double F[2];
    
    // Attractive force
    for (i=0; i<2; i++) {
        F[i] = - path->k_att * (rob_pos_2D[i] - target[i]);
	}
	
	//printf("F_att = (%1.2f, %1.2f)   ", F[0], F[1]);
	
	// Repulsive forces
	for (j=0; j<NB_POINTS_OBSTACLES; j++) {
	    rho = sqrt(pow(rob_pos_2D[0] - path->obstacles[0][j], 2) + pow(rob_pos_2D[1] - path->obstacles[1][j], 2));
	    if (rho < path->obstacles_rho_0[j]) {
	       // printf("Obs %d  ", j);
	        k = path->k_rep[j] * (1.0 / rho - 1.0 / path->obstacles_rho_0[j]) / pow(rho, 3);
	        for (i=0; i<2; i++) {
                F[i] += k * (rob_pos_2D[i] - path->obstacles[i][j]);
            }
	    }
	}
	
	double alpha = atan2(F[1], F[0]) - rob_pos->theta;
	double norm_F = path->speed_coeff * sqrt(F[0]*F[0] + F[1]*F[1]);
	double V = norm_F * cos(alpha);
	double omega = norm_F * sin(alpha);
	
	path->v_l = V - cvs->robot_dimensions->wheel_axle * omega;
	path->v_r = V + cvs->robot_dimensions->wheel_axle * omega;
	
	// printf("F = (%1.2f, %1.2f)   V: %1.2f    omega: %1.2f\n", F[0], F[1], V, omega);
	
	// printf("Time delay: %f\n", cvs->inputs->t - path->t_flag);
	
	if (pow(rob_pos_2D[0] - path->targets[0][index_next_target], 2) + pow(rob_pos_2D[1] - path->targets[1][index_next_target], 2) < 0.01) {
	    if (path->next && (cvs->inputs->t - path->t_flag > 0.5)) {
	        path->next = 0;
	        path->index_next_target += 1;
	        // printf("Next = 0\n");
	    } else if (!path->next) {
	        path->next = 1;
	        path->t_flag = cvs->inputs->t;
	        // printf("Next = 1\n");
	    }
	}
	
}

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif
