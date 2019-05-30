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
void set_init_position(int plus_or_minus, RobotPosition *rob_pos)
{
	printf("Initial position\n");
	
	rob_pos->x = -0.50;
	rob_pos->y = -1.19 * plus_or_minus;
	
	rob_pos->theta = plus_or_minus * M_PI / 2;

	rob_pos->odo_l_wheel_last_angle = 0.0;
	rob_pos->odo_r_wheel_last_angle = 0.0;

	rob_pos->last_t = 0.0;
	
	for (int i = 0; i < 3; i++) { 
		for (int j = 0; j < 3; j++) {
			rob_pos->pos_covariance[i][j] = 0;
		}
	}
	
    rob_pos->pos_covariance_triang[0][0] = 4.865171097055174e-04;
    rob_pos->pos_covariance_triang[1][1] = 0.003475816855762;
    rob_pos->pos_covariance_triang[0][1] = -1.822367507852236e-04;
    rob_pos->pos_covariance_triang[1][0] = rob_pos->pos_covariance_triang[0][1];
}

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif
