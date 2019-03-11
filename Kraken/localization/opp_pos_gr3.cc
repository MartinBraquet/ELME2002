#include "opp_pos_gr3.h"
#include <math.h>

#if ROBOTICS_COURSE
    NAMESPACE_INIT(ctrlGr3);
#endif

/*! \brief compute the opponents position using the tower
 * 
 * \param[in,out] cvs controller main structure
 */
void opponents_tower(CtrlStruct *cvs)
{
	// variables declaration
	int nb_opp; ///< number of opponents

	CtrlIn *inputs; ///< inputs
	RobotPosition *rob_pos; ///< robot own position
	OpponentsPosition *opp_pos; ///< opponents position

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;

	nb_opp = opp_pos->nb_opp;

	// no opponent
	if (!nb_opp)
	{
		return;
	}
}

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif
