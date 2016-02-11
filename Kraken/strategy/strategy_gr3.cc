#include "strategy_gr3.h"
#include "path_planning_gr3.h"
#include "speed_regulation_gr3.h"

#if ROBOTICS_COURSE
    NAMESPACE_INIT(ctrlGr3);
#endif

/*! \brief startegy during the game
 * 
 * \param[in,out] cvs controller main structure
 */
void main_strategy(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;

	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{
		case STRAT_STATE_1:
			path_planning_update(cvs); // update the path-planning

			speed_regulation(cvs, 2.0, 2.0); // send commands to the wheels
			break;

		case STRAT_STATE_2:
			break;

		case STRAT_STATE_3:
			break;

		default:
			printf("Strategy error: unknown state: %d !\n", strat->state);
			exit(EXIT_FAILURE);
	}
}

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif
