#include "../useful/useful.h"
#include "sensor_merging_gr3.h"
#include "init_pos_gr3.h"
#include "../path/path_planning_gr3.h"
#include "lidar.h"
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
	
	//printf("rob_pos->grad_h\n");
	//printMatrix(rob_pos->grad_h, 3, 3);
	
	// Does not merge if the covariance is nul or if the robot is moving
	if(fabs(rob_pos->x_lidar - rob_pos->x) > 0.2 || fabs(rob_pos->y_lidar - rob_pos->y) > 0.2 ) {
		return;
	}
/*
	// precomputation
	double mTemp[3][3], grad_h_P_grad_h_T[3][3], sigma_IN[3][3], sigma_IN_inv[3][3], grad_h_T[3][3];

	transposeMatrix(rob_pos->grad_h, grad_h_T);
	
	// Sigma_IN = grad(h) * P * grad(h)^T + cov_triang
	multiplyMatrices(rob_pos->grad_h, rob_pos->pos_covariance, mTemp, 3, 3, 3, 3);
	multiplyMatrices(mTemp, grad_h_T, grad_h_P_grad_h_T, 3, 3, 3, 3);
	sumMatrices(grad_h_P_grad_h_T, rob_pos->pos_covariance_triang, sigma_IN, 3, 3);
	
	inverseMatrix(sigma_IN, sigma_IN_inv);
	
	// K = P * grad(h)^T / Sigma_in
	double K[3][3];
	multiplyMatrices(rob_pos->pos_covariance, grad_h_T, mTemp, 3, 3, 3, 3);
	multiplyMatrices(mTemp, sigma_IN_inv, K, 3, 3, 3, 3);
	
	//printMatrix(rob_pos->pos_covariance, 3, 3);
	
	double v[3] = {rob_pos->x_lidar - rob_pos->x, rob_pos->y_lidar - rob_pos->y, rob_pos->theta_lidar - rob_pos->theta};
	
	
	if (fabs(v[2]) > M_PI) {
		if (v[2] < 0.0) {
			v[2] += 2.0 * M_PI;
		} else {
			v[2] -= 2.0 * M_PI;
		}
	}
	
	double K_v[3];
	multiplyMatrices_Vector(K, v, K_v);
	*/
	// merging means because the triangulation position has been updated
	//printf("\nMERGE (GUESSED): x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", rob_pos->x, rob_pos->y, rob_pos->theta * 180 / M_PI);
	
	rob_pos->x = (rob_pos->x + rob_pos->x_lidar) / 2.0;
	rob_pos->y = (rob_pos->y + rob_pos->y_lidar) / 2.0;
	
	double sin_theta = (sin(rob_pos->theta) + sin(rob_pos->theta)) / 2.0;
	double cos_theta = (cos(rob_pos->theta) + cos(rob_pos->theta)) / 2.0;
	rob_pos->theta = atan2(sin_theta, cos_theta);
	
	//printf("MERGE (LIDAR): x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", cvs->rob_pos->x_lidar, cvs->rob_pos->y_lidar, cvs->rob_pos->theta_lidar * 180 / M_PI);
	//printf("MERGE (RESULT) : x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", rob_pos->x, rob_pos->y, rob_pos->theta * 180 / M_PI);

/*
    // Merging (co)variances
    
	// Cov = (I - K * grad(h)) * old_cov
	double K_grad_h[3][3];
	double I[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
	multiplyMatrices(K, rob_pos->grad_h, K_grad_h, 3, 3, 3, 3);
	subtractMatrices(I, K_grad_h, mTemp, 3, 3);
	
	double old_covariance[3][3];
	copyMatrix(rob_pos->pos_covariance, old_covariance, 3, 3);
	multiplyMatrices(mTemp, old_covariance, rob_pos->pos_covariance, 3, 3, 3, 3);

	//printf("Sensors merged\n");
*/
}


void Kalman_filter(CtrlStruct *cvs) {

	RobotPosition *rob_pos = cvs->rob_pos;
    PathPlanning *path     = cvs->path;
    LIDAR_data *lidar_data = cvs->lidar_data;
	
	//printf("rob_pos->grad_h\n");
	//printMatrix(rob_pos->grad_h, 3, 3);
	
	// Does not merge if the covariance is nul 
	
	if(rob_pos->pos_covariance[0][0] == 0.0) {
		rob_pos->pos_covariance[0][0] = 0.1;
		rob_pos->pos_covariance[1][1] = 0.1;
		rob_pos->pos_covariance[2][2] = 0.1;
		return;
	}

    //////////////////////////////////////////////////////////
	//          Update phase of Kalman filter               //
	//////////////////////////////////////////////////////////
	
	double mTemp[3][3];
	double grad_h_P_grad_h_T[3][3]; // !!! grad_h_P_grad_h_T is 2x2 !!!
	double sigma_IN[3][3];          // !!! sigma_IN          is 2x2 !!!
	double sigma_IN_inv[3][3];      // !!! sigma_IN_inv      is 2x2 !!!
	double grad_h_T[3][3];          // !!! grad_h_T          is 3x2 !!!
	double grad_h[2][3] = {{0.0, 0.0, -1.0}, {0.0, 0.0, 0.0}};
	double alpha, r;
	double h[2];    // Beacons coordinates in the robot frame
	double K[3][3]; // Kalman gain  // !!! K                 is 3x2 !!!
	double v[3];    // Innovation
	double K_v[3];
	double K_grad_h[3][3];
	double I[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
	double old_covariance[3][3];
	double dy, dx;
	
 	
	for (int i=0; i<NB_BEACONS && lidar_data->lidar_mean_distances[i] < 1.8; i++) {
		
		if (path->beacons[1][i] == 0.0) continue;

		h[0] = atan2(path->beacons[1][i] - rob_pos->y, path->beacons[0][i] - rob_pos->x) - rob_pos->theta;
		h[1] = sqrt(pow(rob_pos->x - path->beacons[0][i], 2) + pow(rob_pos->y - path->beacons[1][i], 2));
		//printf("%d) alpha: %f       r: %f\n", i, h[0] * 180 / M_PI, h[1]);
		
		dy = rob_pos->y - path->beacons[1][i];
		dx = rob_pos->x - path->beacons[0][i];
		grad_h[0][0] = - dy / (dx * dx + dy * dy);
		grad_h[0][1] = - dx / dy * grad_h[0][0];
		grad_h[1][0] =   dx / h[1];
		grad_h[1][1] =   dy / h[1];
		//printMatrix(grad_h, 2, 3);
		
		transposeMatrix(grad_h, grad_h_T);
		
		//printMatrix(grad_h_T, 3, 2);

		// Innovation
		v[0] = lidar_data->lidar_mean_angles[i] - h[0];
		limit_angle(&v[0]);
		v[1] = lidar_data->lidar_mean_distances[i] - h[1];
		//printf("%d) innovation: alpha: %f       r: %f\n", i, v[0] * 180 / M_PI, v[1]);
		
		// Covariance of innovation: Sigma_IN = grad(h) * P * grad(h)^T + cov_triang
		multiplyMatrices(grad_h, rob_pos->pos_covariance, mTemp, 2, 3, 3); // !!! mTemp is 2x3 !!!
		multiplyMatrices(mTemp, grad_h_T, grad_h_P_grad_h_T, 2, 3, 2);
		sumMatrices(grad_h_P_grad_h_T, rob_pos->pos_covariance_triang, sigma_IN, 2, 2);
		
		//printMatrix(sigma_IN, 3, 3);
		
		inverseMatrix22(sigma_IN, sigma_IN_inv);
		
		// K = P * grad(h)^T * Sigma_in^(-1)
		multiplyMatrices(rob_pos->pos_covariance, grad_h_T, mTemp, 3, 3, 2);
		multiplyMatrices(mTemp, sigma_IN_inv, K, 3, 2, 2);
		
		//printMatrix(K, 3, 2);
		
		multiplyMatrices_Vector32(K, v, K_v);
		
		//printf("%d) Update Kalman: d_x: %f    d_y: %f    d_theta: %f\n", i, K_v[0], K_v[1], K_v[2] * 180 / M_PI);
		
		rob_pos->x     = rob_pos->x     + K_v[0];
		rob_pos->y     = rob_pos->y     + K_v[1];
		rob_pos->theta = rob_pos->theta + K_v[2];


		// Merging covariance matrixes
		
		// Cov = (I - K * grad(h)) * old_cov
		multiplyMatrices(K, grad_h, K_grad_h, 3, 2, 3);
		subtractMatrices(I, K_grad_h, mTemp, 3, 3);
		
		copyMatrix(rob_pos->pos_covariance, old_covariance, 3, 3);
		multiplyMatrices(mTemp, old_covariance, rob_pos->pos_covariance, 3, 3, 3);
		
		//printMatrix(mTemp, 3, 3);
		//printMatrix(old_covariance, 3, 3);
		printMatrix(rob_pos->pos_covariance, 3, 3);
		
		//printf("\nMERGE with beacon %d: x:%1.5f [m] ; y:%1.5f [m] ; theta:%1.5f [deg]\n", i, rob_pos->x, rob_pos->y, rob_pos->theta * 180 / M_PI);
		

	}

	//printf("Sensors merged\n");

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
