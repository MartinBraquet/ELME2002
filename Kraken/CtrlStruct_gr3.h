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
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/SPI/Specific/SPI_CAN.hh"
#include "IO/COM/SPI/SPI.hh"
#include "results/saveResult.h"

/// main states
enum {WAIT_INIT_STATE, RUN_STATE, STOP_END_STATE, NB_MAIN_STATES};

/// teams
enum {TEAM_YELLOW, TEAM_PURPLE};

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
typedef struct OpponentsPosition OpponentsPosition;
typedef struct PathPlanning PathPlanning;
typedef struct Strategy Strategy;
typedef struct MotorStruct MotorStruct;
typedef struct LIDAR_data LIDAR_data;
typedef struct SaveFileStruct;

/// Dimensions of the robot
typedef struct Robot_dimensions
{
	double radius;   ///< radius of the robot
	double wheel_radius;   ///< wheel radius of the robot
	double odo_wheel_radius;   ///< wheel radius of the robot
	double wheel_axle;   ///< distance between the wheels
	double lidar_distance;   ///< distance between the lidar and the center of the robot
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
	PathPlanning *path; ///< path-planning
	Strategy *strat; ///< strategy
	MotorStruct *motor_str; ///< motors structure
	Robot_dimensions *robot_dimensions; ///< robot dimensions
	LIDAR_data *lidar_data;
    SaveFileStruct *save;
    
	int main_state; ///< main state
	int plus_or_minus;    ///< ID of the team
	
	int robot_team;    ///< ID of the team
	int main_strategy;    ///< ID of the team
	
	int started;

	int keyboard;    ///< Keyboard mode
	int stop_lidar;    ///< Keyboard mode

	CAN *can;
	SPI *spi;

} CtrlStruct;

// function prototypes
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs);
void free_CtrlStruct(CtrlStruct *cvs);

#endif
