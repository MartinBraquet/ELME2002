#include "speed_regulation_gr3.h"
#include "speed_controller_gr3.h"
#include "CtrlStruct_gr3.h"

#if ROBOTICS_COURSE
    #include "namespace_ctrl.h"
    NAMESPACE_INIT(ctrlGr3);
#endif

/*! \brief wheels speed regulation
 * 
 * \param[in,out] cvs controller main structure
 * \parem[in] r_sp_ref right wheel speed reference [rad/s]
 * \parem[in] l_sp_ref left wheel speed reference [rad/s]
 */
void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref)
{
	// variables declaration
	CtrlIn  *inputs;
	CtrlOut *outputs;
	SpeedRegulation *sp_reg;

	// variables initialization
	inputs  = cvs->inputs;
	outputs = cvs->outputs;
	sp_reg  = cvs->sp_reg;

	// wheel commands
	run_speed_controller(cvs, l_sp_ref, r_sp_ref);

	// last update time
	sp_reg->last_t = inputs->t;
}

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif
