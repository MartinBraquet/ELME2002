#include "calibration_gr3.h"
#include "speed_regulation_gr3.h"

#if ROBOTICS_COURSE
    NAMESPACE_INIT(ctrlGr3);
#endif

// calibration states
enum {CALIB_START, CALIB_STATE_1, CALIB_STATE_2, CALIB_STATE_3, CALIB_FINISH};

/*! \brief calibration of the robot to get its actual position
 * 
 * \param[in,out] cvs controller main structure
 */
void calibration(CtrlStruct *cvs)
{
	// variables declaration
	double t;

	CtrlIn *inputs; ///< controller inputs
	RobotCalibration *calib; ///< calibration structure
	RobotPosition *rob_pos;  ///< robot position (to calibrate)

	// variables initialization
	inputs  = cvs->inputs;
	calib   = cvs->calib;
	rob_pos = cvs->rob_pos;
	
	t = inputs->t;

	// finite state machine (FSM)
	switch (calib->flag)
	{
		case CALIB_START: // start calibration
			speed_regulation(cvs, 0.0, 0.0); // not moving

			calib->flag = CALIB_STATE_1; // directly go to state CALIB_STATE_1
			calib->t_flag = t; // save current time
			break;

		case CALIB_STATE_1: // first calibration state
			speed_regulation(cvs, 3.0, 3.0); // forward speed references

			if (t - calib->t_flag > 2.0) // 2 seconds elapsed since state CALIB_START
			{
				calib->flag = CALIB_STATE_2;
				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_2: // second calibration state
			speed_regulation(cvs, -3.0, -3.0);

			if (t - calib->t_flag > 4.0) // 4 seconds elapsed since state CALIB_STATE_1
			{
				calib->flag = CALIB_STATE_3;
				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_3: // third calibration state
			speed_regulation(cvs, -2.0, -2.0);

			if (t - calib->t_flag > 1.0) // 1 second elapsed since state CALIB_STATE_2
			{
				calib->flag = CALIB_FINISH;
				cvs->main_state = WAIT_INIT_STATE; // tell to the main state that the calibration is done
			}
			break;

		case CALIB_FINISH: // wait before the match is starting
			speed_regulation(cvs, 0.0, 0.0);
			break;
	
		default:
			printf("Error: unknown calibration state : %d !\n", calib->flag);
			exit(EXIT_FAILURE);
	}
}

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif
