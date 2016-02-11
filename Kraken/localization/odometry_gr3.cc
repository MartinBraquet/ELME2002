#include "odometry_gr3.h"
#include "init_pos_gr3.h"
#include <math.h>

#if ROBOTICS_COURSE
    NAMESPACE_INIT(ctrlGr3);
#endif

/*! \brief update the robot odometry
 * 
 * \param[in,out] cvs controller main structure
 */
void update_odometry(CtrlStruct *cvs)
{
	// variables declaration
	CtrlIn *inputs;         ///< controller inputs
	RobotPosition *rob_pos; ///< robot position

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;

	// new odometry position
	rob_pos->x = 0.0;
	rob_pos->y = 0.0;
	rob_pos->theta = 0.0;

	// last update time
	rob_pos->last_t = inputs->t;
}

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif
