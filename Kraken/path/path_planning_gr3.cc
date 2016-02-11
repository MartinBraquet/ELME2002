#include "path_planning_gr3.h"

#if ROBOTICS_COURSE
    NAMESPACE_INIT(ctrlGr3);
#endif

/*! \brief initialize the path-planning algorithm
 * 
 * \param[in,out] cvs controller main structure
 */
void init_path_planning(CtrlStruct *cvs)
{
    int i;
    
    double targets_x[11] = {0, -400, 800, 800, 800, 800, -400, -820, -500, 350, 350};
	double targets_y[11] = {0, -1250, -1300, -550, 550, 1300, 1250, -600, 0, -950, 950};
	for (i=0; i<11; i++) {	
		cvs->path->targets_x[i] = 0.001 * targets_x[i];
		cvs->path->targets_y[i] = 0.001 * targets_y[i];
		printf("Targets x: %f   ; Targets x: %f\n", cvs->path->targets_x[i], cvs->path->targets_y[i]);
	}
	
    //Obstacles
	double obstacles_x[NB_POINTS_OBSTACLES] = {-1060, -900, -800, -700, -600, -500, -400, -300, -200, -100, 0,
	                                               100, 200, 300, 400, 500, 600, 700, 800, 900, 1060,                     // Bottom wall
					1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060,
					    1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060,               // Right wall
					-1060, -900, -800, -700, -600, -500, -400, -300, -200, -100, 0, 100, 200, 300, 400, 500, 
					    600, 700, 800, 900, 1060,                                                                         // Top wall
					-1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060,
	                    -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060, -1060  // Left wall
							                   }; 

	double obstacles_y[NB_POINTS_OBSTACLES] = {-1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560,
	                                               -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560, -1560,         // Bottom wall
					-1400, -1300, -1200, -1100, -1000, -900, -800, -700, -600, -500, -400, -300, -200, -100, 0, 100, 200, 300, 400,
					    500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400,                                            // Right wall
					1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560,
					    1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560,                                             // Top wall
					-1400, -1300, -1200, -1100, -1000, -900, -800, -700, -600, -500, -400, -300, -200, -100,
					    0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400,                     // Left wall
							                   };

	double rho_0 = 150; // Walls
	double w = 1;
	for (i=0; i<NB_POINTS_OBSTACLES; i++)
	{
		cvs->path->obstacles_x[i] = 0.001 * obstacles_x[i];
		cvs->path->obstacles_y[i] = 0.001 * obstacles_y[i];
		cvs->path->obstacles_rho_0[i] = rho_0;
		cvs->path->obstacles_weight[i] = w;
		printf("Obstacle x: %f   ; Obstacle x: %f   ; Obstacle rho_0: %f   ; Obstacle weight: %f\n",
		    cvs->path->obstacles_x[i], cvs->path->obstacles_y[i], cvs->path->obstacles_rho_0[i], cvs->path->obstacles_weight[i]);
	}
}

/*! \brief update the path-planning algorithm
 * 
 * \param[in,out] cvs controller main structure
 */
void path_planning_update(CtrlStruct *cvs)
{
    
}

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif
