#include "strategy_gr3.h"
#include "../path/path_planning_gr3.h"
#include "../localization/init_pos_gr3.h"
#include "../regulation/speed_regulation_gr3.h"


void init_strategy(CtrlStruct *cvs)
{
	cvs->strat->state = STRAT_STATE_1;
	cvs->strat->mini_state = MINI_STRAT_STRAIGHT_LINE;
}

/*! \brief startegy during the game
 * 
 * \param[in,out] cvs controller main structure
 */
void main_strategy(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	PathPlanning *path = cvs->path;

	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{
		case STRAT_STATE_1:
			path_planning_update(cvs); // update the path-planning
			speed_regulation(cvs, path->v_r, path->v_l);
			break;

		case STRAT_STATE_2:
			full_speed(cvs, strat->mini_state);
			break;

		case STRAT_STATE_3:
			round_trip(cvs, strat->mini_state);
			break;

		case STRAT_STATE_4:
			if (cvs->inputs->t > 10.0) {
				obstacles_avoidance(cvs);
			}
			break;

		default:
			printf("Strategy error: unknown state: %d !\n", strat->state);
			exit(EXIT_FAILURE);
	}
}

void round_trip(CtrlStruct *cvs, int mini_state) {

	printf("Mini state: %d\n", mini_state);
	printf("ODOMETER: x = %1.3f ; y = %1.3f ; theta = %1.3f \n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta * 180 / M_PI);
			
	switch (mini_state)
	{

		case  MINI_STRAT_STRAIGHT_LINE:
		
			speed_regulation(cvs, 10.0, 10.0);
			
			if (cvs->rob_pos->y > 0.0) {
				cvs->strat->mini_state = MINI_STRAT_ROTATE_RIGHT;
			}
		
			break;

		case MINI_STRAT_ROTATE_RIGHT:
		
			speed_regulation(cvs, -2.5, 2.5);
			
			if (cvs->rob_pos->theta < 0.0) {
				cvs->strat->mini_state = MINI_STRAT_STRAIGHT_LINE_TOP;
			}
		
			break;

		case MINI_STRAT_STRAIGHT_LINE_TOP:
		
			speed_regulation(cvs, 10.0, 10.0);
			
			if (cvs->rob_pos->x > 0.2) {
				cvs->strat->mini_state = MINI_STRAT_ROTATE_RIGHT2;
			}
		
			break;

		case MINI_STRAT_ROTATE_RIGHT2:
		
			speed_regulation(cvs, -2.5, 2.5);
			
			if (cvs->rob_pos->theta < -M_PI/2) {
				cvs->strat->mini_state = MINI_STRAT_STRAIGHT_LINE_BACK;
			}
		
			break;

		case MINI_STRAT_STRAIGHT_LINE_BACK:
		
			speed_regulation(cvs, 10.0, 10.0);
			
			if (cvs->rob_pos->y < -1.0) {
				cvs->strat->mini_state = MINI_STRAT_WAIT_END;
			}
		
			break;

		case MINI_STRAT_ROTATE_LEFT:
		
			speed_regulation(cvs, 2.5, -2.5);
			
			if (cvs->rob_pos->theta > M_PI/2) {
				cvs->strat->mini_state = MINI_STRAT_STRAIGHT_LINE;
			}
		
			break;

		case MINI_STRAT_WAIT_END:
		
			speed_regulation(cvs, 0.0, 0.0);
		
			break;

		default:
			printf("Mini state error: unknown state: %d !\n", mini_state);
			exit(EXIT_FAILURE);
	}
	
}

void full_speed(CtrlStruct *cvs, int mini_state) {

	//printf("Mini state: %d\n", mini_state);

	switch (mini_state)
	{

		case  MINI_STRAT_STATE_1:
		
			speed_regulation(cvs, 10.0, 10.0);

			if (cvs->inputs->t > 5.0) {
				cvs->strat->mini_state = MINI_STRAT_STATE_2;
			}
		
			break;

		case  MINI_STRAT_STATE_2:
		
			speed_regulation(cvs, 30.0, 30.0);
			
			if (cvs->rob_pos->y > 1.0) {
				cvs->strat->mini_state = MINI_STRAT_STATE_3;
				printf("Time full speed: %f\n", cvs->inputs->t);
			}
		
			break;

		case MINI_STRAT_STATE_3:
		
			speed_regulation(cvs, 0, 0);
		
			break;


		default:
			printf("Mini state error: unknown state: %d !\n", mini_state);
			exit(EXIT_FAILURE);
	}
	
}

void obstacles_avoidance(CtrlStruct *cvs) {
	
	CtrlIn *inputs = cvs->inputs;
	double V, W;
	double speed = 5.0;
	
	if (inputs->opp_detected) {
		V = - speed * cos(inputs->relative_theta_opp);
		W = - speed * sin(inputs->relative_theta_opp);
	} else {
		V = 0.0;
		W = 0.0;
	}
	
	printf("V: %f   W: %f", V, W);
	
	speed_regulation(cvs, V + W, V - W);
	
}
