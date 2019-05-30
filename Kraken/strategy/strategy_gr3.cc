#include "strategy_gr3.h"
#include "../path/path_planning_gr3.h"
#include "../localization/init_pos_gr3.h"
#include "../localization/lidar.h"
#include "../regulation/speed_regulation_gr3.h"
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>


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

	switch (cvs->main_strategy)
	{
		case 0:
			strategy_1(cvs, strat->mini_state);
			break;

		case 1:
			strategy_1(cvs, strat->mini_state);
			break;

		case 2:
			strategy_1(cvs, strat->mini_state);
			break;

		case 3:
			strategy_1(cvs, strat->mini_state);
			break;

		case 4:
			full_speed(cvs, strat->mini_state);
			break;

		case 5:
			round_trip(cvs, strat->mini_state);
			break;

		case 6:
			if (cvs->inputs->t > 10.0) {
				obstacles_avoidance(cvs);
			}
			break;
/*
		case STRAT_STATE_5:
			path->targets[0][0] = 0.2; path->targets[1][0] = -0.2;
			path->targets[0][1] = -0.5; path->targets[1][1] = -0.2;
			path->targets[0][2] = -0.5; path->targets[1][2] = -1.0;
			path->targets[0][3] = 0.2; path->targets[1][3] = -1.0;
			if (path->index_next_target > 3)
				path->index_next_target -= 4;
			path_planning_update(cvs); // update the path-planning
			speed_regulation(cvs, path->v_r, path->v_l);
			break;
*/
		default:
			printf("Strategy error: unknown state: %d !\n", strat->state);
			exit(EXIT_FAILURE);
	}
}

void strategy_1(CtrlStruct *cvs, int mini_state) {
	
	fprintf(cvs->save->fichierSpeedWheel, "%0.5f, %0.5f , %0.5f, %0.5f, %0.5f\n",cvs->inputs->t,cvs->path->v_l,cvs->inputs->motor_enc_l_wheel_speed,cvs->path->v_r,cvs->inputs->motor_enc_r_wheel_speed);
	fprintf(cvs->save->fichierPostionRobot, "%0.5f, %0.5f , %0.5f, %0.5f, %0.5f, %0.5f \n",cvs->inputs->t,cvs->rob_pos->x,cvs->rob_pos->y,cvs->rob_pos->theta,cvs->path->targets[0][cvs->path->index_next_target],cvs->path->targets[1][cvs->path->index_next_target]);
	
	//printf("Mini state: %d\n", mini_state);
	
	path_planning_update(cvs); // update the path-planning
	
	speed_regulation(cvs, cvs->path->v_r, cvs->path->v_l);
	
	printf("V: (%f, %f)\n", cvs->path->v_l, cvs->path->v_r);
	
	
	
	//if (cvs->lidar_data->stop) {
		//speed_regulation(cvs, 0.0, 0.0);
	//} else {
	//}
	
	//speed_regulation(cvs, cvs->path->v_r, cvs->path->v_l);
	
	/*
	//To sent the score after a particular task was done
	switch (cvs->path->index_next_target) 
	{
		case 4: 
			//Here we bring back the palet from the chaos zone
			//And one or two in front of the box
			cvs->outputs->score = 13; 
		#if USE_LT24
			//RW_data_LT24(cvs, 0)
		#endif
			break;
		default: 
			break;
	}
	* */
	
	/*
	switch (mini_state)
		{
			
			case 0:
				if (abs(cvs->rob_pos->theta) < M_PI/100)
					cvs->strat->mini_state++;
				else
					speed_regulation(cvs, 5, 10);
				break;
			// straigth ahead
			case 0:
				if (abs(cvs->rob_pos->y) <= 0.88)
					cvs->strat->mini_state++;
				else
					speed_regulation(cvs, 5, 5);
				break;
			case 1:
				if (cvs->rob_pos->x > 0.0)
					cvs->strat->mini_state++;
				else
					speed_regulation(cvs, cvs->plus_or_minus*5, cvs->plus_or_minus*5);
				break;
			case 2:
				if (abs(cvs->rob_pos->theta + M_PI/2) < M_PI/50)
					cvs->strat->mini_state++;
				else
					speed_regulation(cvs, -cvs->plus_or_minus*2.5, cvs->plus_or_minus*2.5);
				break;
			case 3:
				if (abs(cvs->rob_pos->y) > 1.0)
					cvs->strat->mini_state++;
				else
					speed_regulation(cvs, cvs->plus_or_minus*2.5, cvs->plus_or_minus*2.5);
				break;
			case 4:
				if (abs(cvs->rob_pos->y) < 0.9)
					cvs->strat->mini_state++;
				else
					speed_regulation(cvs, -cvs->plus_or_minus*2.5, -cvs->plus_or_minus*2.5);
				break;
			case 5:
				if (abs(cvs->rob_pos->theta - M_PI) < M_PI/50)
					cvs->strat->mini_state++;
				else
					speed_regulation(cvs, -cvs->plus_or_minus*2.5, cvs->plus_or_minus*2.5);
				break;
			// move to wall and then take atoms
			case 6:
				if (cvs->rob_pos->x > 0.4) {
					cvs->strat->mini_state++;
					cvs->outputs->next_action = 1;
					sem_t *actions_semaphore;
					actions_semaphore = sem_open("/actions_semaphore", O_CREAT);
					sem_post(actions_semaphore);
				}
				else
					speed_regulation(cvs, -cvs->plus_or_minus*2.5, -cvs->plus_or_minus*2.5);
				break;
			// wait for end of action
			case 7:
				if (cvs->outputs->action_in_progress == 0) {
					cvs->strat->mini_state++;
				}
				break;
			case 8:
				printf("%d\n", cvs->path->index_next_target);
				path_planning_update(cvs); // update the path-planning
				speed_regulation(cvs, cvs->path->v_r, cvs->path->v_l);
				break;
				
			// turn to wall
			case 1:
				if (abs(cvs->rob_pos->theta - M_PI) <= M_PI/50)
					cvs->strat->mini_state++;
				else
					speed_regulation(cvs, cvs->plus_or_minus*2.5, -cvs->plus_or_minus*2.5);
				break;
			// go to wall
			case 2:
				if (abs(cvs->rob_pos->x) > 0.4){
					cvs->strat->mini_state++;
					cvs->outputs->next_action = 1;
					
					sem_t *actions_semaphore;
					actions_semaphore = sem_open("/actions_semaphore", O_CREAT);
					sem_post(actions_semaphore);
				}
				else
					speed_regulation(cvs, -5, -5);
				break;
			// wait for end of action
			case 3:
				if (cvs->outputs->action_in_progress == 0)
					cvs->strat->mini_state++;
				break;
			// go backwards
			case 4:
				if (abs(cvs->rob_pos->x) < 0.2)
					cvs->strat->mini_state++;
				else
					speed_regulation(cvs, 2.5, 2.5);
				break;
			// turn
			case 5:
				if (cvs->rob_pos->theta < 0)
					cvs->strat->mini_state++;
				else
					speed_regulation(cvs, 2.5, 2.5);
				break;

			case 5:*/
				
				/*break;
			default:
				printf("Strategy error: unknown state: %d !\n", cvs->strat->mini_state);
				exit(EXIT_FAILURE);
		}	*/
	
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
	LIDAR_data *lidar_data= cvs->lidar_data;
	double V, W;
	double speed = 5.0;
	
	if (lidar_data->opp_detected) {
		V = - speed * cos(lidar_data->relative_theta_opp);
		W = - speed * sin(lidar_data->relative_theta_opp);
	} else {
		//V = 0.0;
		//W = 0.0;
	}
	
	printf("V: %f   W: %f", V, W);
	
	speed_regulation(cvs, V + W, V - W);
	
}
