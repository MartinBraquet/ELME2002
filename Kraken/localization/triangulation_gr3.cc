#include "triangulation_gr3.h"
#include "../path/path_planning_gr3.h"
#include "../useful/useful.h"
#include "../path/path_planning_gr3.h"
#include "../strategy/strategy_gr3.h"
#include "../CtrlStruct_gr3.h"
#include "init_pos_gr3.h"
#include "lidar.h"
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

int cmp_with_indexes(const void *a, const void *b)
{
    struct str *a1 = (struct str *)a;
    struct str *a2 = (struct str *)b;
    if ((*a1).distance > (*a2).distance)
        return 1;
    else if ((*a1).distance < (*a2).distance)
        return -1;
    else
        return 0;
}

void triangulation_mean_data(CtrlStruct *cvs) {
  
    RobotPosition *rob_pos = cvs->rob_pos;
    PathPlanning *path = cvs->path;
    OpponentsPosition *opp_pos = cvs->opp_pos;
    CtrlIn *inputs = cvs->inputs;
    LIDAR_data *lidar_data = cvs->lidar_data;
    int i,j,stop;
    
    double pos_lidar[2];
    int n[NB_BEACONS] = {0,0,0,0,0};
    double sin_a[NB_BEACONS] = {0.0,0.0,0.0,0.0,0.0};
    double cos_a[NB_BEACONS] = {0.0,0.0,0.0,0.0,0.0};
    double d[NB_BEACONS] = {0.0,0.0,0.0,0.0,0.0};
    double n_opp = 0.0;
    double sin_opp = 0.0;
    double cos_opp = 0.0;
    double d_opp = 0.0;
    
    lidar_data->stop = 0;
    
    for (i = 0; i < inputs->lidar_count; i++) {
        stop = 0;
        pos_lidar[0] = rob_pos->x + inputs->lidar_distances[i] * cos(rob_pos->theta + inputs->lidar_angles[i]);
        pos_lidar[1] = rob_pos->y + inputs->lidar_distances[i] * sin(rob_pos->theta + inputs->lidar_angles[i]);
        //printf("%1.3f, %1.3f\n", pos_lidar[0], pos_lidar[1]);
        
        for (j = 0; j < NB_BEACONS; j++) {
          // Beacon j position
          if (abs(pos_lidar[0] - path->beacons[0][j]) < BEACONS_SQUARE && abs(pos_lidar[1] - path->beacons[1][j]) < BEACONS_SQUARE) {
              //printf("LIDAR: Beacon %d (%1.3f, %1.3f)  dist: %1.3f\n", j, pos_lidar[0], pos_lidar[1], inputs->lidar_distances[i]);
              //printf("%f, %f\n", inputs->lidar_angles[i], inputs->lidar_distances[i]);
              n[j]++;
              sin_a[j] += sin(inputs->lidar_angles[i]);
              cos_a[j] += cos(inputs->lidar_angles[i]);
              d[j] += inputs->lidar_distances[i];
              stop = 1;
              break;
          }
        } 
    
        // Opponent
        if (!stop && inputs->lidar_distances[i] < path->opp_rho_0 && abs(pos_lidar[0]) < MAP_X1-BEACONS_SQUARE &&  abs(pos_lidar[1]) < MAP_Y1-BEACONS_SQUARE && inputs->lidar_distances[i] > 0.2) {
           //printf("LIDAR: OPP (%1.3f, %1.3f)  angle: %1.3f  dist: %1.3f\n", pos_lidar[0], pos_lidar[1], inputs->lidar_angles[i] * 180 / M_PI, inputs->lidar_distances[i]);
           n_opp++;
           limit_angle(&inputs->lidar_angles[j]);
           sin_opp += sin(inputs->lidar_angles[i]);
           cos_opp += cos(inputs->lidar_angles[i]);
           d_opp += inputs->lidar_distances[i];
           lidar_data->stop = 1;
        }
    }
    
    // Means for beacons
    for (j=0; j<NB_BEACONS; j++) {
      if (n[j] > 0) {
        lidar_data->lidar_mean_angles[j]    = atan2(sin_a[j]/n[j], cos_a[j]/n[j]);
        lidar_data->lidar_mean_distances[j] = d[j]/n[j];
        limit_angle(&lidar_data->lidar_mean_angles[j]);
        //printf("LIDAR: %d angle %f dist %f\n", j, lidar_data->lidar_mean_angles[j] * 180 / M_PI, lidar_data->lidar_mean_distances[j]);
      } else {
        lidar_data->lidar_mean_distances[j] = 999.999;
      }
    }
    
    double theta_opp = rob_pos->theta + atan2(sin_opp/n_opp, cos_opp/n_opp);
    limit_angle(&theta_opp);
    
    if (n_opp != 0) {
      lidar_data->relative_theta_opp = atan2(sin_opp/n_opp, cos_opp/n_opp);
      lidar_data->dist_opp = d_opp / n_opp;
      //printf("LIDAR: OPP FINAL theta: %1.3f, d: %1.3f\n", lidar_data->relative_theta_opp * 180 / M_PI, lidar_data->dist_opp);
      opp_pos->x[0] =  rob_pos->x + d_opp/n_opp * cos(theta_opp);
      opp_pos->y[0] =  rob_pos->y + d_opp/n_opp * sin(theta_opp);
    }
    
    //printf("LIDAR: OPP FINAL (%1.3f, %1.3f)\n", opp_pos->x[0], opp_pos->y[0]);
    
    struct str distances_and_indexes[NB_BEACONS];
    for (i = 0; i < NB_BEACONS; i++)
    {
        distances_and_indexes[i].distance = lidar_data->lidar_mean_distances[i];
        distances_and_indexes[i].index = i;
    }
    qsort(distances_and_indexes, NB_BEACONS, sizeof(distances_and_indexes[0]), cmp_with_indexes);
    
    for (i = 0; i < NB_BEACONS; i++)
    {
        lidar_data->nearest_indexes[i] = distances_and_indexes[i].index;
    }
    //printf("lidar_mean_distances: %f %f %f %f\n", lidar_data->lidar_mean_distances[lidar_data->nearest_indexes[0]], lidar_data->lidar_mean_distances[lidar_data->nearest_indexes[1]], lidar_data->lidar_mean_distances[lidar_data->nearest_indexes[2]], lidar_data->lidar_mean_distances[lidar_data->nearest_indexes[3]]);
    //printf("index_nearest_beacons: %d %d %d\n", lidar_data->nearest_indexes[0], lidar_data->nearest_indexes[1], lidar_data->nearest_indexes[2]);
  
    
}

/*! \brief finds the position thanks to the beacons
 * 
 * \param[in,out] cvs controller main structure
 */
void triangulation(CtrlStruct *cvs)
{
    RobotPosition *rob_pos = cvs->rob_pos;
    LIDAR_data *lidar_data = cvs->lidar_data;
    
    int index[3] = {0, 1, 2};
    
    triangulation_angles(cvs, index, &rob_pos->x_lidar, &rob_pos->y_lidar, &rob_pos->theta_lidar);
    //printf("LIDAR triangulation_angles: x = %1.3f ; y = %1.3f ; theta = %1.3f \n", rob_pos->x_lidar, rob_pos->y_lidar, rob_pos->theta_lidar * 180 / M_PI);
      
    
    if(lidar_data->lidar_mean_distances[lidar_data->nearest_indexes[3]] < 2.0) {
      index[2] = 3;
      double x, y, sin_theta, cos_theta, theta;
      triangulation_angles(cvs, index, &x, &y, &theta);
      rob_pos->x_lidar = 2.0 / 3.0 * rob_pos->x_lidar + x / 3.0;
      rob_pos->y_lidar = 2.0 / 3.0 * rob_pos->y_lidar + y / 3.0;
      sin_theta = 2.0 / 3.0 * sin(rob_pos->theta_lidar) + sin(theta) / 3.0;
      cos_theta = 2.0 / 3.0 * cos(rob_pos->theta_lidar) + cos(theta) / 3.0;
      
      rob_pos->theta_lidar = atan2(sin_theta, cos_theta);
      //printf("LIDAR triangulation_angles MERGED: x = %1.3f ; y = %1.3f ; theta = %1.3f \n", rob_pos->x_lidar, rob_pos->y_lidar, rob_pos->theta_lidar * 180 / M_PI);
      //printf("%f %f %f\n", rob_pos->x_lidar, rob_pos->y_lidar, rob_pos->theta_lidar);
    }
    
}
  
  
void triangulation_angles(CtrlStruct *cvs, int index[3], double *x, double *y, double *theta)
{
    RobotPosition *rob_pos = cvs->rob_pos;
    PathPlanning *path     = cvs->path;
    LIDAR_data *lidar_data = cvs->lidar_data;
    
    CtrlIn *inputs = cvs->inputs;
    int i;
      
    double beacons[2][3];
    double theta_beacon[3];
    
    for (i=0; i<3; i++) {
      theta_beacon[i] = lidar_data->lidar_mean_angles[lidar_data->nearest_indexes[index[i]]] + rob_pos->theta;
      beacons[0][i]   = path->beacons[0][lidar_data->nearest_indexes[index[i]]];
      beacons[1][i]   = path->beacons[1][lidar_data->nearest_indexes[index[i]]];
    }
    
    //printf("theta0: %1.3f ; theta1: %1.3f ; theta2: %1.3f\n", theta_beacon[0] * 180/M_PI, theta_beacon[1] * 180/M_PI, theta_beacon[2] * 180/M_PI);
    
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
        *x = beacons[0][1] + k31_p * (y12_p - y23_p) / D;
        *y = beacons[1][1] + k31_p * (x23_p - x12_p) / D;
        *theta = atan2(beacons[1][0] - *y, beacons[0][0] - *x) - theta_beacon[0] + rob_pos->theta;
        limit_angle(theta);
        *x -= cvs->robot_dimensions->lidar_distance * cos(*theta);
        *y -= cvs->robot_dimensions->lidar_distance * sin(*theta);
    } else {
        //printf("TRIANGULATION: D = 0 !\n");
    }
    
    //printf("LIDAR triangulation_angles: x = %1.3f ; y = %1.3f ; theta = %1.3f\n", *x, *y, *theta * 180 / M_PI);
    
    /*
    double d_T12_d_phi1 =   1.0 / pow(sin(theta_beacon[1] - theta_beacon[0]), 2);
    double d_T12_d_phi2 = - 1.0 / pow(sin(theta_beacon[1] - theta_beacon[0]), 2);
    double d_T23_d_phi2 =   1.0 / pow(sin(theta_beacon[2] - theta_beacon[1]), 2);
    double d_T23_d_phi3 = - 1.0 / pow(sin(theta_beacon[2] - theta_beacon[1]), 2);
    double d_T31_d_phi1 = - d_T12_d_phi1 * (T23 * T23 + 1.0) / pow(T12 + T23, 2);
    double d_T31_d_phi2 = - (d_T12_d_phi2 * (T23 * T23 + 1.0) + d_T23_d_phi2 * (T12 * T12 + 1.0)) / pow(T12 + T23, 2);
    double d_T31_d_phi3 = - d_T23_d_phi3 * (T12 * T12 + 1.0) / pow(T12 + T23, 2);
    
    double d_x12_d_phi1 =   y1_p * d_T12_d_phi1;
    double d_x12_d_phi2 =   y1_p * d_T12_d_phi2;
    double d_x23_d_phi2 = - y3_p * d_T23_d_phi2;
    double d_x23_d_phi3 = - y3_p * d_T23_d_phi3;
    double d_x31_d_phi1 = (y3_p - y1_p) * d_T31_d_phi1;
    double d_x31_d_phi2 = (y3_p - y1_p) * d_T31_d_phi2;
    double d_x31_d_phi3 = (y3_p - y1_p) * d_T31_d_phi3;
    double d_y12_d_phi1 = - x1_p * d_T12_d_phi1;
    double d_y12_d_phi2 = - x1_p * d_T12_d_phi2;
    double d_y23_d_phi2 =   x3_p * d_T23_d_phi2;
    double d_y23_d_phi3 =   x3_p * d_T23_d_phi3;
    double d_y31_d_phi1 = (x1_p - x3_p) * d_T31_d_phi1;
    double d_y31_d_phi2 = (x1_p - x3_p) * d_T31_d_phi2;
    double d_y31_d_phi3 = (x1_p - x3_p) * d_T31_d_phi3;
    
    double d_D_d_phi1 = d_x12_d_phi1 * (y23_p - y31_p) - (x12_p - x23_p) * d_y31_d_phi1 - d_y12_d_phi1 * (x23_p - x31_p) + (y12_p - y23_p) * d_x31_d_phi1;
    double d_D_d_phi2 = (d_x12_d_phi2 - d_x23_d_phi2) * (y23_p - y31_p) + (x12_p - x23_p) * (d_y23_d_phi2 - d_y31_d_phi2) - (d_y12_d_phi2 - d_y23_d_phi2) * (x23_p - x31_p) - (y12_p - y23_p) * (d_x23_d_phi2 - d_x31_d_phi2);
    double d_D_d_phi3 = - d_x23_d_phi3 * (y23_p - y31_p) - (x12_p - x23_p) * d_y31_d_phi3 + d_y23_d_phi3 * (x23_p - x31_p) + (y12_p - y23_p) * d_x31_d_phi3;
    
    double d_k31_d_phi1 = d_T31_d_phi1 * (x1_p * y3_p - x3_p * y1_p);
    double d_k31_d_phi2 = d_T31_d_phi2 * (x1_p * y3_p - x3_p * y1_p);
    double d_k31_d_phi3 = d_T31_d_phi3 * (x1_p * y3_p - x3_p * y1_p);
    */
    
    /*
    GRADIENT OF H: the transformation from (phi1, phi2, phi3) to (x, y, theta)
               _                                        _
              | d_x_d_phi1   d_y_d_phi1   d_theta_d_phi1 |
    grad_h =  | d_x_d_phi2   d_y_d_phi2   d_theta_d_phi2 |
              | d_x_d_phi3   d_y_d_phi3   d_theta_d_phi3 |
               ‾                                        ‾
    */
    /*
    grad_h[0][0] = d_k31_d_phi1 * (y12_p - y23_p) / D + k31_p * d_y12_d_phi1 / D - k31_p * (y12_p - y23_p) * d_D_d_phi1 / (D * D);
    grad_h[1][0] = d_k31_d_phi2 * (y12_p - y23_p) / D + k31_p * (d_y12_d_phi2 - d_y23_d_phi2) / D - k31_p * (y12_p - y23_p) * d_D_d_phi2 / (D * D);
    grad_h[2][0] = d_k31_d_phi3 * (y12_p - y23_p) / D - k31_p * d_y23_d_phi3 / D - k31_p * (y12_p - y23_p) * d_D_d_phi3 / (D * D);
    
    grad_h[0][1] = d_k31_d_phi1 * (x23_p - x12_p) / D - k31_p * d_x12_d_phi1 / D - k31_p * (x23_p - x12_p) * d_D_d_phi1 / (D * D);
    grad_h[1][1] = d_k31_d_phi2 * (x23_p - x12_p) / D + k31_p * (d_x23_d_phi2 - d_x12_d_phi2) / D - k31_p * (x23_p - x12_p) * d_D_d_phi2 / (D * D);
    grad_h[2][1] = d_k31_d_phi3 * (x23_p - x12_p) / D + k31_p * d_x23_d_phi3 / D - k31_p * (x23_p - x12_p) * d_D_d_phi3 / (D * D);
    
    grad_h[0][2] = (grad_h[0][1] * (rob_pos->x_lidar - path->beacons[0][0]) + grad_h[0][0] * (path->beacons[1][0] - rob_pos->y_lidar)) / (pow(path->beacons[1][0] - rob_pos->y_lidar, 2) + pow(path->beacons[0][0] - rob_pos->x_lidar, 2)) - 1.0;
    grad_h[1][2] = (grad_h[1][1] * (rob_pos->x_lidar - path->beacons[0][0]) + grad_h[1][0] * (path->beacons[1][0] - rob_pos->y_lidar)) / (pow(path->beacons[1][0] - rob_pos->y_lidar, 2) + pow(path->beacons[0][0] - rob_pos->x_lidar, 2));
    grad_h[2][2] = (grad_h[2][1] * (rob_pos->x_lidar - path->beacons[0][0]) + grad_h[2][0] * (path->beacons[1][0] - rob_pos->y_lidar)) / (pow(path->beacons[1][0] - rob_pos->y_lidar, 2) + pow(path->beacons[0][0] - rob_pos->x_lidar, 2));
	  */
}

void triangulation_distances_all_three(CtrlStruct *cvs) {
      
    RobotPosition *rob_pos = cvs->rob_pos;
    PathPlanning *path = cvs->path;
    LIDAR_data *lidar_data = cvs->lidar_data;
    
    CtrlIn *inputs = cvs->inputs;
    int i;
    
    double x0 = path->beacons[0][lidar_data->nearest_indexes[0]];
    double y0 = path->beacons[1][lidar_data->nearest_indexes[0]];
    double x1 = path->beacons[0][lidar_data->nearest_indexes[1]];
    double y1 = path->beacons[1][lidar_data->nearest_indexes[1]];
    double x2 = path->beacons[0][lidar_data->nearest_indexes[2]];
    double y2 = path->beacons[1][lidar_data->nearest_indexes[2]];

    double d0 = lidar_data->lidar_mean_distances[lidar_data->nearest_indexes[0]];
    double d1 = lidar_data->lidar_mean_distances[lidar_data->nearest_indexes[1]];
    double d2 = lidar_data->lidar_mean_distances[lidar_data->nearest_indexes[2]];

    double d01_square = d1 * d1 - d0 * d0;
    double d02_square = d2 * d2 - d0 * d0;
    double xy0_square = x0 * x0 + y0 * y0;
    double xy1_square = x1 * x1 + y1 * y1;
    double xy2_square = x2 * x2 + y2 * y2;

    double x01 = x1 - x0;
    double y01 = y1 - y0;
    double y02 = y2 - y0;
    double x02 = x2 - x0;

    double x = 0.5 * ((d01_square - xy1_square + xy0_square) * y02 - (d02_square - xy2_square + xy0_square) * y01) / (x02 * y01 - x01 * y02);
    double y = (0.5 * (-d02_square - xy0_square + xy2_square) - x * x02) / y02;

    //printf("LIDAR triangulation_distances_all_three: x = %1.3f ; y = %1.3f\n", x, y);
  
}

void triangulation_distances_two_nearest(CtrlStruct *cvs) {
  
    RobotPosition *rob_pos = cvs->rob_pos;
    PathPlanning *path = cvs->path;
    LIDAR_data *lidar_data = cvs->lidar_data;
    
    CtrlIn *inputs = cvs->inputs;
    int i;
    
    double x0 = path->beacons[0][lidar_data->nearest_indexes[0]];
    double y0 = path->beacons[1][lidar_data->nearest_indexes[0]];
    double x1 = path->beacons[0][lidar_data->nearest_indexes[1]];
    double y1 = path->beacons[1][lidar_data->nearest_indexes[1]];
    double x2 = path->beacons[0][lidar_data->nearest_indexes[2]];
    double y2 = path->beacons[1][lidar_data->nearest_indexes[2]];

    double d0 = lidar_data->lidar_mean_distances[lidar_data->nearest_indexes[0]];
    double d1 = lidar_data->lidar_mean_distances[lidar_data->nearest_indexes[1]];
    double d2 = lidar_data->lidar_mean_distances[lidar_data->nearest_indexes[2]];

    double x01 = x1 - x0;
    double y01 = y1 - y0;

    double theta = acos((d0 * d0 + x01 * x01 + y01 * y01 - d1 * d1) / (2.0 * d0 * sqrt(x01 * x01 + y01 * y01)));
                   
    double beta = atan2(y01,x01);

    double x_guess1 = x0 + d0 * cos(beta + theta);
    double y_guess1 = y0 + d0 * sin(beta + theta);

    double dist_x_guess1 = x_guess1 - x2;
    double dist_y_guess1 = y_guess1 - y2;
    double dist_guess1_squared = dist_y_guess1 * dist_y_guess1 + dist_x_guess1 * dist_x_guess1;

    double x_guess2 = x0 + d0 * cos(beta - theta);
    double y_guess2 = y0 + d0 * sin(beta - theta);

    double dist_x_guess2 = x_guess2 - x2;
    double dist_y_guess2 = y_guess2 - y2;
    double dist_guess2_squared = dist_y_guess2 * dist_y_guess2 + dist_x_guess2 * dist_x_guess2;

    double x, y;
    if (abs(d2 * d2 - dist_guess2_squared) > abs(d2 * d2 - dist_guess1_squared)) {
      x = x_guess1;
      y = y_guess1;
    } else {
      x = x_guess2;
      y = y_guess2;
    }

    //printf("LIDAR triangulation_distances_two_nearest: x = %1.3f ; y = %1.3f\n", x, y);
    //printf("x: %lf; y: %lf; theta: %lf; beta: %lf\n", x, y, theta, beta);
}

