#include "odometry_gr3.h"
#include "init_pos_gr3.h"
#include "../useful/useful.h"
#include <math.h>

#define kl 0.0001
#define kr 0.0001

/*! \brief update the robot odometry
 * 
 * \param[in,out] cvs controller main structure
 */
void update_odometry(CtrlStruct *cvs)
{
	// variables declaration
	CtrlIn *inputs;         ///< controller inputs
	RobotPosition *rob_pos; ///< robot position
	Robot_dimensions *robot_dims;

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;
	robot_dims = cvs->robot_dimensions;

	//printf("odo_l_wheel_last_angle: %1.3f ; inputs->odo_r_wheel_angle: %1.3f\n", rob_pos->odo_l_wheel_last_angle, inputs->odo_r_wheel_angle);

	double delta_s_l = robot_dims->odo_wheel_radius * (inputs->odo_l_wheel_angle - rob_pos->odo_l_wheel_last_angle);
	double delta_s_r = robot_dims->odo_wheel_radius * (inputs->odo_r_wheel_angle - rob_pos->odo_r_wheel_last_angle);
    
	double delta_s     = (delta_s_r + delta_s_l) / 2.0; 
	double delta_theta = (delta_s_r - delta_s_l) / robot_dims->wheel_axle;
	
	//printf("%f ........................\n", robot_dims->odo_wheel_radius); 

	// new odometry position
	rob_pos->x = rob_pos->x + delta_s * cos(rob_pos->theta + delta_theta / 2.0);
	rob_pos->y = rob_pos->y + delta_s * sin(rob_pos->theta + delta_theta / 2.0);
	rob_pos->theta = rob_pos->theta + delta_theta;
	limit_angle(&rob_pos->theta);

	// Error matrixes for old position contribution
	double sin_thetal_l = sin(rob_pos->theta);
	double cos_thetal_l = cos(rob_pos->theta);
	
	//printf("cos_thetal_l: %f      sin_thetal_l: %f\n", cos_thetal_l, sin_thetal_l);

	double Fp[3][3] = {{1.0,	0.0, -delta_s * sin_thetal_l},			/// jacobian	; M 3x3
					  {0.0,	1.0,  delta_s * cos_thetal_l},
					  {0.0, 0.0,			1.0			}};
	
	double Fp_T[3][3];
	transposeMatrix(Fp, Fp_T);

	double Mp[][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};		/// current position covariance contribution; M 3x3

    double wheel_axle = robot_dims->wheel_axle;
	double Frl[3][3] = { {0.5*cos_thetal_l - delta_s * sin_thetal_l / (2.0 * wheel_axle),
	                     0.5*cos_thetal_l + delta_s * sin_thetal_l / (2.0 * wheel_axle), 0.0},
						{0.5*sin_thetal_l + delta_s * cos_thetal_l / (2.0 * wheel_axle),
						 0.5*sin_thetal_l - delta_s * cos_thetal_l / (2.0 * wheel_axle), 0.0},
						{1.0 / wheel_axle, -1.0 / wheel_axle, 0.0}};		/// increment jacobian ; M 3x2

	double Frl_T[3][3];
	transposeMatrix(Frl, Frl_T);

	double Erl[][3] = {{kr*abs(delta_s_r), 0.0,              0.0},		/// covariance matrix of the increment; M 3x3
  					   {0.0,              kl*abs(delta_s_l), 0.0},
	                   {0.0,              0.0,              0.0}};

	double Mrl[][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};		/// increment covariance contribution; M 3x3

	double mTemp[][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

	
	// update error: Fp * pos_covariance * Fp_T
	multiplyMatrices(Fp, rob_pos->pos_covariance, mTemp, 3, 3, 3);
	multiplyMatrices(mTemp, Fp_T, Mp, 3, 3, 3);
	
    // update error: Frl * Erl * Mrl
	multiplyMatrices(Frl, Erl, mTemp, 3, 2, 2);
	multiplyMatrices(mTemp, Frl_T, Mrl, 3, 2, 3);
	
	//printMatrix(Mp, 3, 3);
	//printMatrix(Mrl, 3, 3);

    // pos_covariance = Fp * pos_covariance * Fp_T + Frl * Erl * Mrl
	sumMatrices(Mp, Mrl, rob_pos->pos_covariance, 3, 3);

	//printMatrix(rob_pos->pos_covariance, 3, 3);
	
	// last update time
	rob_pos->last_t = inputs->t;
	rob_pos->odo_l_wheel_last_angle = inputs->odo_l_wheel_angle;
	rob_pos->odo_r_wheel_last_angle = inputs->odo_r_wheel_angle;
}
