#include "init_pos_gr3.h"
#include "../CtrlStruct_gr3.h"
#include <math.h>

#if ROBOTICS_COURSE
    NAMESPACE_INIT(ctrlGr3);
#endif

/*! \brief set the initial robot position (guess if noise added)
 * 
 * \param[in] robot_id robot ID
 * \param[out] rob_pos robot position structure
 */
void set_init_position(int robot_team, RobotPosition *rob_pos)
{
	switch (robot_team)
	{
		case TEAM_YELLOW:
			rob_pos->x = 0.0;
			rob_pos->y = -1.37;
			rob_pos->theta = M_PI / 2;
			break;

		case TEAM_PURPLE:
			rob_pos->x = 0.0;
			rob_pos->y = 1.37;
			rob_pos->theta = -M_PI / 2;
			break;
	
		default:
			printf("Initial position error: unknown robot team: %d !\n", robot_team);
			exit(EXIT_FAILURE);
	}	

	rob_pos->x_odometer = rob_pos->x;
	rob_pos->y_odometer = rob_pos->y;
	rob_pos->theta_odometer  = rob_pos->theta;

	rob_pos->odo_l_wheel_last_angle = 0.0;
	rob_pos->odo_r_wheel_last_angle = 0.0;

	rob_pos->last_t = 0.0;
	
}

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif
