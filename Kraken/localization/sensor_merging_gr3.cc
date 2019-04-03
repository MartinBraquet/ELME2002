#include "../useful/useful.h"
#include "sensor_merging_gr3.h"
#include "init_pos_gr3.h"
#include <math.h>
#include <stdio.h>

int checkPositionUpdateNeeded(CtrlStruct *cvs) {

    RobotPosition *rob_pos = cvs->rob_pos;
    /*
	double error_x_independent     = fabs(rob_pos->x_beacons     - rob_pos->x_odometer);
	double error_y_independent     = fabs(rob_pos->y_beacons     - rob_pos->y_odometer);
	double error_theta_independent = fabs(rob_pos->theta_beacons - rob_pos->theta_odometer);

	double error_x_combined     = fabs(rob_pos->x_beacons     - rob_pos->x);
	double error_y_combined     = fabs(rob_pos->y_beacons     - rob_pos->y);
	double error_theta_combined = fabs(rob_pos->theta_beacons - rob_pos->theta);
    */

	if (rob_pos->pos_covariance[0][0] > LIM_VAR_X)
		return SENSOR_UPDATE_NEEDED;
	else if (rob_pos->pos_covariance[1][1] > LIM_VAR_Y)
		return SENSOR_UPDATE_NEEDED;
	else if (rob_pos->pos_covariance[2][2] > LIM_VAR_THETA)
		return SENSOR_UPDATE_NEEDED;
	/*
	else if (error_x_combined > error_x_independent + LIM_ERR_X_COMB)
		return SENSOR_RECOMBINATION_NEEDED;
	else if (error_y_combined > error_y_independent + LIM_ERR_Y_COMB)
		return SENSOR_RECOMBINATION_NEEDED;
	else if (error_theta_combined > error_theta_independent + LIM_ERR_THETA_COMB)
		return SENSOR_RECOMBINATION_NEEDED;
	*/
	else
		return SENSOR_UPDATE_NOT_NEEDED;
}


void mergeSensorData(CtrlStruct *cvs) {
	// from the reference book notation, I am assuming
	// k->odometer
	// z->

	RobotPosition *rob_pos = cvs->rob_pos;

	// precomputation
	double k_x	   = rob_pos->pos_covariance[0][0] / (rob_pos->pos_covariance[0][0] + rob_pos->pos_covariance_triang[0][0]);
	double k_y	   = rob_pos->pos_covariance[1][1] / (rob_pos->pos_covariance[1][1] + rob_pos->pos_covariance_triang[1][1]);
	double k_theta = rob_pos->pos_covariance[2][2] / (rob_pos->pos_covariance[2][2] + rob_pos->pos_covariance_triang[2][2]);
		
	// merging means because the triangulation position has been updated
	printf("MERGE (GUESSED): x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", rob_pos->x, rob_pos->y, rob_pos->theta * 180 / M_PI);
	if (fabs(rob_pos->x_lidar - rob_pos->x) + fabs(rob_pos->y_lidar - rob_pos->y) < 0.2) {
	    rob_pos->x     = rob_pos->x     + k_x     * (rob_pos->x_lidar     - rob_pos->x);
	    rob_pos->y     = rob_pos->y     + k_y     * (rob_pos->y_lidar     - rob_pos->y);
	    if (fabs(rob_pos->theta_lidar - rob_pos->theta) > M_PI) {
		if (rob_pos->theta_lidar < rob_pos->theta) {
		    rob_pos->theta = rob_pos->theta + k_theta * (2.0 * M_PI + rob_pos->theta_lidar - rob_pos->theta);
		} else {
		    rob_pos->theta = rob_pos->theta + k_theta * (-2.0 * M_PI + rob_pos->theta_lidar - rob_pos->theta);
		}
	    } else {
		rob_pos->theta = rob_pos->theta + k_theta * (rob_pos->theta_lidar - rob_pos->theta);
	    }
	}
	printf("MERGE (BEACONS): x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", cvs->rob_pos->x_lidar, cvs->rob_pos->y_lidar, cvs->rob_pos->theta_lidar * 180 / M_PI);
	printf("MERGE (RESULT) : x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", rob_pos->x, rob_pos->y, rob_pos->theta * 180 / M_PI);

	// merging (co)variances
	rob_pos->pos_covariance[0][0] = rob_pos->pos_covariance[0][0] * (1 - k_x);
	rob_pos->pos_covariance[1][1] = rob_pos->pos_covariance[1][1] * (1 - k_y);
	rob_pos->pos_covariance[2][2] = rob_pos->pos_covariance[2][2] * (1 - k_theta);
	rob_pos->pos_covariance[0][1] = 0;
	rob_pos->pos_covariance[0][2] = 0;
	rob_pos->pos_covariance[1][0] = 0;
	rob_pos->pos_covariance[1][2] = 0;
	rob_pos->pos_covariance[2][0] = 0;
	rob_pos->pos_covariance[2][1] = 0;

	//printf("Sensors merge\n");

}
/*
void recombineSensorData(CtrlStruct *cvs) {
	// from the reference book notation, I am assuming
	// k->odometer
	// z->

	RobotPosition *rob_pos = cvs->rob_pos;

	//pre computation
	double k_x = rob_pos->pos_covariance[0][0] / (rob_pos->pos_covariance[0][0] + rob_pos->pos_covariance_triang[0][0]);
	double k_y = rob_pos->pos_covariance[1][1] / (rob_pos->pos_covariance[1][1] + rob_pos->pos_covariance_triang[1][1]);
	double k_theta = rob_pos->pos_covariance[2][2] / (rob_pos->pos_covariance[2][2] + rob_pos->pos_covariance_triang[2][2]);

	// merging means
	rob_pos->x += k_x * (rob_pos->x_lidar - rob_pos->x_odometer);
	rob_pos->y += k_y * (rob_pos->y_lidar - rob_pos->y_odometer);
	rob_pos->theta += k_theta * (rob_pos->theta_lidar - rob_pos->theta_odometer);

	// merging (co)variances
	rob_pos->pos_covariance[0][0] = rob_pos->pos_covariance[0][0] * (1 - k_x);
	rob_pos->pos_covariance[1][1] = rob_pos->pos_covariance[1][1] * (1 - k_y);
	rob_pos->pos_covariance[2][2] = rob_pos->pos_covariance[2][2] * (1 - k_theta);
	rob_pos->pos_covariance[0][1] = 0;
	rob_pos->pos_covariance[0][2] = 0;
	rob_pos->pos_covariance[1][0] = 0;
	rob_pos->pos_covariance[1][2] = 0;
	rob_pos->pos_covariance[2][0] = 0;
	rob_pos->pos_covariance[2][1] = 0;

	printf("\n\n\n\n\n\n\n\n\n\n\n\nI HAVE JUST RECOMBINED THE SENSORS!!!\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
}
*/
