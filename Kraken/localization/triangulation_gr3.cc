#include "triangulation_gr3.h"
#include "../path/path_planning_gr3.h"
#include "../useful/limit_angle_gr3.h"
#include "../path/path_planning_gr3.h"
#include "../CtrlStruct_gr3.h"
#include "init_pos_gr3.h"
#include <math.h>
#include <stdio.h>

#define PRINT_TRIANGULATION 0


int cmp(const void *x, const void *y)
{
  double xx = *(double*)x, yy = *(double*)y;
  if (xx < yy) return -1;
  if (xx > yy) return  1;
  return 0;
}

/*! \brief finds the position thanks to the beacons
 * 
 * \param[in,out] cvs controller main structure
 */
void triangulation(CtrlStruct *cvs)
{
    RobotPosition *rob_pos = cvs->rob_pos;
    PathPlanning *path = cvs->path;
    
    CtrlIn *inputs = cvs->inputs;
    int i;
    
/*
    if (PRINT_TRIANGULATION) {
        printf("last_rising_fixed : ");
        for (i=0; i<NB_STORE_EDGE; i++) {
            printf("%1.3f   ", inputs->last_rising_fixed[i] * 180 / M_PI);
        }
        
        printf("\nlast_falling_fixed: ");
        for (i=0; i<NB_STORE_EDGE; i++) {
            printf("%1.3f   ", inputs->last_falling_fixed[i] * 180 / M_PI);
        }
        
        printf("\nnb_rising_fixed: %d\n", inputs->nb_rising_fixed);
        printf("nb_rising_fixed: %d\n", inputs->nb_falling_fixed);
        printf("rising_index_fixed: %d\n", inputs->rising_index_fixed);
        printf("falling_index_fixed: %d\n", inputs->falling_index_fixed);
    }
    */
    double theta_beacon[3];
    for (i=0; i<3; i++) {
        theta_beacon[i] = cvs->rob_pos->theta;
    }
    
    /*
    int last_edge[3] = {2, 0, 1};
    double angle_rising, angle_falling; 
    
    for (i=0; i<3; i++) {
        if (inputs->falling_index_fixed == inputs->rising_index_fixed) {
            angle_rising = inputs->last_rising_fixed[(inputs->rising_index_fixed - i + NB_STORE_EDGE) % NB_STORE_EDGE];
            angle_falling = inputs->last_falling_fixed[(inputs->falling_index_fixed - i + NB_STORE_EDGE) % NB_STORE_EDGE];
        } else {
            theta_beacon[i] += (inputs->last_rising_fixed[(inputs->rising_index_fixed - i + NB_STORE_EDGE) % NB_STORE_EDGE] 
                          + inputs->last_falling_fixed[(inputs->falling_index_fixed - last_edge[i] + NB_STORE_EDGE) % NB_STORE_EDGE]) / 2.0;
        }
        if (angle_rising - angle_falling > M_PI) {
            angle_falling += 2 * M_PI;
        }
        theta_beacon[i] += (angle_rising + angle_falling) / 2.0;
        limit_angle(&theta_beacon[i]);
        
        /*
        if (abs(angle_rising - angle_falling) > 1.0) {
            printf("Error angle: %1.3f   %1.3f   theta = %1.3f\n", angle_rising * 180 / M_PI, angle_falling * 180 / M_PI, theta_beacon[i] * 180 / M_PI);
        }
        */
    //}
    
    qsort(theta_beacon, sizeof(theta_beacon)/sizeof(theta_beacon[0]), sizeof(theta_beacon[0]), cmp);
    
    if (PRINT_TRIANGULATION) {printf("\ntheta0: %1.3f            ; theta1: %1.3f               ; theta2: %1.3f\n", theta_beacon[0] * 180/M_PI, theta_beacon[1] * 180/M_PI, theta_beacon[2] * 180/M_PI);}
    
    // Source: http://www.telecom.ulg.ac.be/publi/publications/pierlot/Pierlot2014ANewThree/
    
    double x1_p = path->beacons[0][0] - path->beacons[0][1];
    double y1_p = path->beacons[1][0] - path->beacons[1][1];
    double x3_p = path->beacons[0][2] - path->beacons[0][1];
    double y3_p = path->beacons[1][2] - path->beacons[1][1];
    
    double T12 = 1.0 / tan(theta_beacon[1] - theta_beacon[0]);
    double T23 = 1.0 / tan(theta_beacon[2] - theta_beacon[1]);
    double T31 = (1.0 - T12 * T23) / (T12 + T23);
    
    double x12_p = x1_p + T12 * y1_p;
    double y12_p = y1_p - T12 * x1_p;
    double x23_p = x3_p - T23 * y3_p;
    double y23_p = y3_p + T23 * x3_p;
    double x31_p = (x3_p + x1_p) + T31 * (y3_p - y1_p);
    double y31_p = (y3_p + y1_p) - T31 * (x3_p - x1_p);
    
    double k31_p = x1_p * x3_p + y1_p * y3_p + T31 * (x1_p * y3_p - x3_p * y1_p);
    
    double D = (x12_p - x23_p) * (y23_p - y31_p) - (y12_p - y23_p) * (x23_p - x31_p);
    
    // printf("TRIANGULATION: D = %f", D);
    
    if (abs(D) > 1e-6) {
        rob_pos->x_beacons = path->beacons[0][1] + k31_p * (y12_p - y23_p) / D;
        rob_pos->y_beacons = path->beacons[1][1] + k31_p * (x23_p - x12_p) / D;
        rob_pos->theta_beacons = atan2(path->beacons[1][0] - rob_pos->y_beacons, path->beacons[0][0] - rob_pos->x_beacons) - (theta_beacon[0] - cvs->rob_pos->theta);
        rob_pos->x_beacons -= cvs->robot_dimensions->tower_distance * cos(rob_pos->theta_beacons);
        rob_pos->y_beacons -= cvs->robot_dimensions->tower_distance * sin(rob_pos->theta_beacons);
    } else {
        printf("TRIANGULATION: D = 0 !\n");
    }
	
	return;
}

