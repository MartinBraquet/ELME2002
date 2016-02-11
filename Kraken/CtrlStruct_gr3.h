/*! 
 * \file CtrlStruct_gr3.h
 * \brief Controller main structure
 */

#ifndef _CTRL_STRUCT_GR3_H_
#define _CTRL_STRUCT_GR3_H_

#define ROBOTICS_COURSE 0 // 0 for Kraken

#include "ctrl_io.h"
#include <stdlib.h>
#include <stdio.h>

#if ROBOTICS_COURSE
    #include "user_realtime.h"
    #include "namespace_ctrl.h"
    NAMESPACE_INIT(ctrlGr3);
#else
    #include <wiringPi.h>
    #include <wiringPiSPI.h>
    #include "IO/COM/CAN/CAN.hh"
    #include "IO/COM/SPI/Specific/SPI_CAN.hh"
    #include "IO/COM/SPI/SPI.hh"
#endif

/// main states
enum {CALIB_STATE, WAIT_INIT_STATE, RUN_STATE, STOP_END_STATE, NB_MAIN_STATES};

/// robot IDs
enum {ROBOT_B, ROBOT_R, ROBOT_Y, ROBOT_W, NB_ROBOTS};

/// teams
enum {TEAM_A, TEAM_B, NB_TEAMS};

//Bus can
#define CAN_BR 500e3

//Bus SPI
#define channel 0
#define clockSpi 500000
//register data in the FPGA
#define LEFT_M_ENC 0x04;
#define RIGHT_M_ENC 0x05;

//RPI header
#define resetPin 25    //GPIO_O_PI[0]

// forward declaration
typedef struct RobotPosition RobotPosition;
typedef struct SpeedRegulation SpeedRegulation;
typedef struct RobotCalibration RobotCalibration;
typedef struct OpponentsPosition OpponentsPosition;
typedef struct PathPlanning PathPlanning;
typedef struct Strategy Strategy;
typedef struct MotorStruct MotorStruct;

/// Dimensions of the robot
typedef struct Robot_dimensions
{
	double radius;   ///< radius of the robot
	double wheel_radius;   ///< wheel radius of the robot
	double wheel_axle;   ///< distance between the wheels
	double beacon_distance;   ///< distance between the beacon and the center
	double beacon_radius;   ///< radius of the beacon
	double microswitch_distance;   ///< distance between one micro-switch and the center
} Robot_dimensions;

/// Main controller structure
typedef struct CtrlStruct
{
	CtrlIn *inputs;   ///< controller inputs
	CtrlOut *outputs; ///< controller outputs

	RobotPosition *rob_pos; ///< robot position
	OpponentsPosition *opp_pos; ///< opponents position
	SpeedRegulation *sp_reg; ///< speed regulation
	RobotCalibration *calib; ///< calibration
	PathPlanning *path; ///< path-planning
	Strategy *strat; ///< strategy
	MotorStruct *motor_str; ///< motors structure
	Robot_dimensions *robot_dimensions; ///< robot dimensions

	int main_state; ///< main state
	int robot_id;   ///< ID of the robot
	int team_id;    ///< ID of the team

	CAN *can;
	SPI *spi;

} CtrlStruct;

// function prototypes
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs);
void free_CtrlStruct(CtrlStruct *cvs);

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif

#endif
