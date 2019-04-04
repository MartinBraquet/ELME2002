#include "triangulation_gr3.h"
#include "../path/path_planning_gr3.h"
#include "../useful/useful.h"
#include "../path/path_planning_gr3.h"
#include "../strategy/strategy_gr3.h"
#include "../CtrlStruct_gr3.h"
#include "init_pos_gr3.h"
#include "opp_pos_gr3.h"
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

void index_nearest_beacons(double distances[5], int ind_near_beac[3]) {
  for (int i = 1; i < 5; i++) {
    if (distances[i] < distances[ind_near_beac[0]]) {
      ind_near_beac[2] = ind_near_beac[1];
      ind_near_beac[1] = ind_near_beac[0];
      ind_near_beac[0] = i;
    } else if (distances[i] < distances[ind_near_beac[1]]) {
      ind_near_beac[2] = ind_near_beac[1];
      ind_near_beac[1] = i;
    } else if (distances[i] < distances[ind_near_beac[2]] && i != 1) {
      ind_near_beac[2] = i;
    }
  }
}

void triangulation_mean_data(CtrlStruct *cvs) {
  
    RobotPosition *rob_pos = cvs->rob_pos;
    PathPlanning *path = cvs->path;
    OpponentsPosition *opp_pos = cvs->opp_pos;
    CtrlIn *inputs = cvs->inputs;
    int i,j,stop;
    
    double pos_lidar[2];
    int n[NB_BEACONS] = {0,0,0,0,0};
    double a[NB_BEACONS] = {0.0,0.0,0.0,0.0,0.0};
    double d[NB_BEACONS] = {0.0,0.0,0.0,0.0,0.0};
    double n_opp = 0.0;
    double sin_opp = 0.0;
    double cos_opp = 0.0;
    double d_opp = 0.0;
    
    for (i = 0; i < LIDAR_SAMPLES; i++) {
        stop = 0;
        pos_lidar[0] = rob_pos->x + inputs->lidar_distances[i] * cos(rob_pos->theta + inputs->lidar_angles[i]);
        pos_lidar[1] = rob_pos->y + inputs->lidar_distances[i] * sin(rob_pos->theta + inputs->lidar_angles[i]);
        //printf("%1.3f, %1.3f\n", pos_lidar[0], pos_lidar[1]);
        
        for (j = 0; j < NB_BEACONS; j++) {
          // Beacon j position
          if (abs(pos_lidar[0] - path->beacons[0][j]) < BEACONS_SQUARE && abs(pos_lidar[1] - path->beacons[1][j]) < BEACONS_SQUARE) {
              //printf("LIDAR: Beacon %d (%1.3f, %1.3f)  dist: %1.3f\n", j, pos_lidar[0], pos_lidar[1], inputs->lidar_distances[i]);
              n[j]++;
              a[j] += inputs->lidar_angles[i];
              d[j] += inputs->lidar_distances[i];
              stop = 1;
              break;
          }
        } 
        
        // Opponent
        if (((!stop && cvs->strat->state != STRAT_STATE_4 && abs(pos_lidar[0]) < MAP_X1-0.2 &&  abs(pos_lidar[1]) < MAP_Y1-0.2) || (cvs->strat->state == STRAT_STATE_4 && inputs->lidar_distances[i] < 0.5)) && inputs->lidar_distances[i] > 0.15) {
           printf("LIDAR: OPP (%1.3f, %1.3f)  angle: %1.3f  dist: %1.3f\n", pos_lidar[0], pos_lidar[1], inputs->lidar_angles[i] * 180 / M_PI, inputs->lidar_distances[i]);
           n_opp++;
           limit_angle(&inputs->lidar_angles[j]);
           sin_opp += sin(inputs->lidar_angles[i]);
           cos_opp += cos(inputs->lidar_angles[i]);
           d_opp += inputs->lidar_distances[i];
        }
    }
    
    for (j=0; j<NB_BEACONS; j++) {
        inputs->lidar_mean_angles[j]    = rob_pos->theta + a[j]/n[j];
        inputs->lidar_mean_distances[j] = d[j]/n[j];
        limit_angle(&inputs->lidar_mean_angles[j]);
        //printf("LIDAR: %d angle %f dist %f\n", j, inputs->lidar_mean_angles[j], inputs->lidar_mean_distances[j]);
    }
    
    double theta_opp = rob_pos->theta + atan2(sin_opp/n_opp, cos_opp/n_opp);
    limit_angle(&theta_opp);
    
    
    if (n_opp != 0) {
      inputs->opp_detected = 1;
      inputs->relative_theta_opp = atan2(sin_opp/n_opp, cos_opp/n_opp);
      inputs->dist_opp = d_opp / n_opp;
      printf("LIDAR: OPP FINAL theta: %1.3f, d: %1.3f\n", inputs->relative_theta_opp * 180 / M_PI, inputs->dist_opp);
      opp_pos->x[0] =  rob_pos->x + d_opp/n_opp * cos(theta_opp);
      opp_pos->y[0] =  rob_pos->y + d_opp/n_opp * sin(theta_opp);
    } else {
      inputs->opp_detected = 0;
    }
    
    //printf("LIDAR: OPP FINAL (%1.3f, %1.3f)\n", opp_pos->x[0], opp_pos->y[0]);
    
}

/*! \brief finds the position thanks to the beacons
 * 
 * \param[in,out] cvs controller main structure
 */
void triangulation_angles(CtrlStruct *cvs)
{
    RobotPosition *rob_pos = cvs->rob_pos;
    PathPlanning *path = cvs->path;
    
    CtrlIn *inputs = cvs->inputs;
    int i;
    
    int ind_near_beac[3] = {0, 1, 2};
    index_nearest_beacons(inputs->lidar_mean_distances, ind_near_beac);
    
    printf("lidar_mean_distances: %f %f %f\n", inputs->lidar_mean_distances[ind_near_beac[0]], inputs->lidar_mean_distances[ind_near_beac[1]], inputs->lidar_mean_distances[ind_near_beac[2]]);
    printf("index_nearest_beacons: %d %d %d\n", ind_near_beac[0], ind_near_beac[1], ind_near_beac[2]);
    
    double beacons[2][3];
    double theta_beacon[3];
    
    for (i=0; i<3; i++) {
      theta_beacon[i] = inputs->lidar_mean_angles[ind_near_beac[i]];
      beacons[0][i]   = path->beacons[0][ind_near_beac[i]];
      beacons[1][i]   = path->beacons[1][ind_near_beac[i]];
    }
    
    printf("\ntheta0: %1.3f            ; theta1: %1.3f               ; theta2: %1.3f\n", theta_beacon[0] * 180/M_PI, theta_beacon[1] * 180/M_PI, theta_beacon[2] * 180/M_PI);
    
    // Source: http://www.telecom.ulg.ac.be/publi/publications/pierlot/Pierlot2014ANewThree/
    
    double x1_p = beacons[0][0] - beacons[0][1];
    double y1_p = beacons[1][0] - beacons[1][1];
    double x3_p = beacons[0][2] - beacons[0][1];
    double y3_p = beacons[1][2] - beacons[1][1];
    
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
        rob_pos->x_lidar = beacons[0][1] + k31_p * (y12_p - y23_p) / D;
        rob_pos->y_lidar = beacons[1][1] + k31_p * (x23_p - x12_p) / D;
        rob_pos->theta_lidar = atan2(beacons[1][0] - rob_pos->y_lidar, beacons[0][0] - rob_pos->x_lidar) - (theta_beacon[0] - rob_pos->theta);
        rob_pos->x_lidar -= cvs->robot_dimensions->lidar_distance * cos(rob_pos->theta_lidar);
        rob_pos->y_lidar -= cvs->robot_dimensions->lidar_distance * sin(rob_pos->theta_lidar);
    } else {
        printf("TRIANGULATION: D = 0 !\n");
    }
    
    printf("LIDAR: x = %1.3f ; y = %1.3f ; theta = %1.3f \n", rob_pos->x_lidar, rob_pos->y_lidar, rob_pos->theta_lidar * 180 / M_PI);
	
	return;
}

void triangulation_distances_all_three(CtrlStruct *cvs) {
  
  double x0 = -1.0;
  double y0 = -1.5;
  double x1 =  1.0;
  double y1 = -1.5;
  double x2 =  0.0;
  double y2 =  1.5;
  
  double d0 = 1.8;
  double d1 = 1.8;
  double d2 = 1.5;

  double d01_square = d1 * d1 - d0 * d0;
  double d02_square = d2 * d2 - d0 * d0;
  double xy0_square = x0 * x0 + y0 * y0;
  double xy1_square = x1 * x1 + y1 * y1;
  double xy2_square = x2 * x2 + y2 * y2;
  
  double y01 = y1 - y0;
  double y02 = y2 - y0;
  double x02 = x2 - x0;

  double x = 0.5 * ((d01_square - xy1_square + xy0_square) * y02 - (d02_square - xy2_square + xy0_square) * y01) 
                      / (x02 * y01 - (x1-x0) * y02);
  double y = (0.5 * (-d02_square - xy0_square + xy2_square) - x * x02) / y02;
  
  printf("%f %f \n", x, y);
}

void triangulation_distances_two_nearest(CtrlStruct *cvs) {
  
  double x0 = -1.0;
  double y0 = -1.5;
  double x1 =  1.0;
  double y1 = -1.5;
  double x2 =  0.0;
  double y2 =  1.5;
  double x3 = -1.0;
  double y3 =  0.0;
  double x4 =  1.0;
  double y4 =  0.0;
  
  double d0 = 2.5;
  double d1 = 1.5;
  double d2 = 1.8;
  double d3 = 5.0;
  double d4 = 5.0;
  
  double distances[5] = {d0, d1, d2, d3, d4};
  double xv[5] = {x0, x1, x2, x3, x4};
  double yv[5] = {y0, y1, y2, y3, y4};
  
  int ind_near_beac[3] = {0, 1, 2};
  index_nearest_beacons(distances, ind_near_beac);
  
  printf("%d %d %d \n", ind_near_beac[0], ind_near_beac[1], ind_near_beac[2]);
  
  double x01 = xv[ind_near_beac[1]] - xv[ind_near_beac[0]];
  double y01 = yv[ind_near_beac[1]] - yv[ind_near_beac[0]];
  
  double theta = acos((distances[ind_near_beac[0]] * distances[ind_near_beac[0]] 
                 + x01 * x01 + y01 * y01 - distances[ind_near_beac[1]] * distances[ind_near_beac[1]])
                 / (2.0 * distances[ind_near_beac[0]] * sqrt(x01 * x01 + y01 * y01)));
                 
  double beta = atan2(y01,x01);
  
  double x_guess1 = xv[ind_near_beac[0]] + distances[ind_near_beac[0]] * cos(beta + theta);
  double y_guess1 = yv[ind_near_beac[0]] + distances[ind_near_beac[0]] * sin(beta + theta);
  
  double dist_x_guess1 = x_guess1 - xv[ind_near_beac[2]];
  double dist_y_guess1 = y_guess1 - yv[ind_near_beac[2]];
  double dist_guess1_squared = dist_y_guess1 * dist_y_guess1 + dist_x_guess1 * dist_x_guess1;
  
  double x_guess2 = xv[ind_near_beac[0]] + distances[ind_near_beac[0]] * cos(beta - theta);
  double y_guess2 = yv[ind_near_beac[0]] + distances[ind_near_beac[0]] * sin(beta - theta);
  
  double dist_x_guess2 = x_guess2 - xv[ind_near_beac[2]];
  double dist_y_guess2 = y_guess2 - yv[ind_near_beac[2]];
  double dist_guess2_squared = dist_y_guess2 * dist_y_guess2 + dist_x_guess2 * dist_x_guess2;
  
  double x, y;
  if (abs(distances[ind_near_beac[2]] * distances[ind_near_beac[2]] - dist_guess2_squared) 
    > abs(distances[ind_near_beac[2]] * distances[ind_near_beac[2]] - dist_guess1_squared)) {
    x = x_guess1;
    y = y_guess1;
  } else {
    x = x_guess2;
    y = y_guess2;
  }
  
  printf("x: %lf; y: %lf; theta: %lf; beta: %lf\n", x, y, theta, beta);
}

