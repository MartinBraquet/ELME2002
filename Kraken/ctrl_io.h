/*!
 * \file ctrl_io.h
 * \brief Structures defining the inputs and the outputs of the Minibot controller
 */

#ifndef _CTRL_IO_H_
#define _CTRL_IO_H_

#include <sweep/sweep.h>

// number of micro-switches
#define NB_U_SWITCH 2

// number of stored rising/falling edges
#define NB_STORE_EDGE 10

#define LIDAR_SAMPLES 1000

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

	/*! \brief tower
	 * 
	 * The tower saves the position of rising and falling edges detected with the laser as a position
	 * relative to the robot. The position is given in radians in the ]-pi ; pi] interval, counter-clockwise.
	 *  -pi/2 corresponds to an edge detected on the right of the robot
	 *      0 corresponds to an edge detected just in front of the robot
	 *   pi/2 corresponds to an edge detected on the left of the robot
	 *     pi corresponds to an edge detected just behind the robot
	 *
	 * The tower must be able to detect more than one beacon, corresponding to more than one rising (and falling)
	 * edge detected during a single rotation. To this end, the relative positions of the last 10 (= NB_STORE_EDGE)
	 * rising and the last 10 falling edges are saved.
	 *
	 * This is stored in two rotating lists:
	 *   the first edge is stored at index 0
	 *   the second is stored at index 1
	 *   ...
	 *   the 10th is stored at index 9
	 *   the 11th is stored at index 0 (erasing the first one)
	 *   the 12th is stored at index 1 (erasing the second one)
	 *   ...
	 *
	 * To this end, an integer indicates the index of the last edge added in the rotating list.
	 *
	 * Finally, the number of rising (and falling) edges detected during the last laser revolution are saved
	 * in two additional integer values. To this end, a reference laser position must be defined to update this
	 * value and to re-start the count of edges.
	 */
	double tower_pos; ///< tower current relative position [rad] in the range ]-pi/2 ; pi/2]

	double last_rising[NB_STORE_EDGE];  ///< rotating list with the last rising edges detected [rad]
	double last_falling[NB_STORE_EDGE]; ///< rotating list with the last falling edges detected [rad]


	// LIDAR data
	sweep_error_s error;
	sweep_device_s sweep;
	double lidar_angles[LIDAR_SAMPLES];
	double lidar_distances[LIDAR_SAMPLES];
	int lidar_signals[LIDAR_SAMPLES];

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

	/*! \brief tower command
	 *
	 * The tower command is similar to the one of the wheels ('wheel_commands'), with the same duty cycle limit corresponding
	 * to a range of [-0.9*24 V ; +0.9*24 V]. However, this motor is too powerful for this tower. Consequently, it is
	 * better to limit 'tower_command' in a range of [-15 ; 15].
	 */
	double tower_command; ///< tower motor command [-], bounded in [-100 ; 100]

} CtrlOut;

#endif
