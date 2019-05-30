/*!
 * \file ctrl_io.h
 * \brief Structures defining the inputs and the outputs of the Minibot controller
 */

#ifndef _CTRL_IO_H_
#define _CTRL_IO_H_

#include <stdlib.h>
#include "rplidar.h"

// number of micro-switches
#define NB_U_SWITCH 2

// number of stored rising/falling edges
#define NB_STORE_EDGE 10

#define LIDAR_SAMPLES 2000

// ID of the right and left sides
enum{R_ID, L_ID};

/// Controller inputs
typedef struct CtrlIn
{
	/*! \brief time reference
	 */
	double t; ///< time [s]

	/*! \brief wheel speeds
	 *
	 * Each wheel speed is positive when the robot is going forward (the front part of the robot is
	 * the round part).
	 */
	double motor_enc_l_wheel_speed; ///< motor ancoder left wheel speed [rad/s]
	double motor_enc_r_wheel_speed; ///< motor ancoder right wheel speed [rad/s]

 	double motor_enc_l_wheel_angle; ///< motor ancoder left wheel angle [rad]
 	double motor_enc_r_wheel_angle; ///< motor ancoder right wheel angle [rad]

	double odo_l_wheel_speed; ///< odometer left wheel speed [rad/s]
	double odo_r_wheel_speed; ///< odometer right wheel speed [rad/s]

 	double odo_l_wheel_angle; ///< odometer left wheel angle [rad]
 	double odo_r_wheel_angle; ///< odometer right wheel angle [rad]

	/*! \brief micro-switches
	 */
	int u_switch[NB_U_SWITCH]; ///< 1 if corresponding u_switch (R_ID or L_ID) is activated, 0 otherwise

	int start_RPI; ///< 0 if the jumper is connected to the interface card

	// LIDAR data
	double lidar_angles[LIDAR_SAMPLES];
	double lidar_distances[LIDAR_SAMPLES];
	int lidar_signals[LIDAR_SAMPLES];
	int lidar_count;
	
	rp::standalone::rplidar::RPlidarDriver *drv;

} CtrlIn;

/// Controller outputs
typedef struct CtrlOut
{
	/*! \brief wheel commands
	 *
	 * Each wheel is commanded by a voltage in the range [-24 V ; +24 V]. To avoid duty cycles problems, it is better
	 * to work in the range [-0.9*24 V ; +0.9*24 V].
	 *
	 * 'wheel_commands' is a command signal bounded in [-100 ; 100], proportional to the voltage sent to the corresponding
	 * wheel motor. Here are three examples of voltages corresponding to 'wheel_commands' values:
	 *   -100 corresponds to -0.9*24 V
	 *      0 corresponds to       0 V
	 *    100 corresponds to +0.9*24 V
	 */
	double wheel_commands[2]; ///< wheel motors (R_ID or L_ID) commands [-], bounded in [-100 ; 100]
	int pneumatic_commands;
	unsigned char next_action;
	unsigned char action_in_progress;
	unsigned char action_finished;
	
	int score;

} CtrlOut;

#endif
