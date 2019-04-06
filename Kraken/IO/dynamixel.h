/*************************************************************************
* Header file for Dynamixel                                            *
* Version 1.00                                                           *
**************************************************************************/
#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_
/*************************************************************************
* Library declarations				                                     *
**************************************************************************/
//#include  <stdio.h>
//#include  <io.h>
//#include  <unistd.h>
//#include "system.h"

#include	<stdio.h>
#include	<time.h>
#include	<math.h>
#include	<unistd.h>			// for UART
#include	<wiringPi.h>
#include	<wiringSerial.h>

#include 	<fcntl.h> 			// for UART
#include	<termios.h>			// for UART

/*************************************************************************
* Types Definitions					                                     *
**************************************************************************/
typedef unsigned char Byte; //integer of 8 bits

/*************************************************************************
* UART Parameters					                                     *
**************************************************************************/
#define UART_DEVICE 		"/dev/serial0"
#define UART_BAULDRATE 		57600
#define UART_DIRECTION_PORT	0				// weird, but corresponds to GPIO17
/*************************************************************************
* User defines						                                     *
**************************************************************************/
#define ON 		1
#define OFF 	0

#define DYNAMIXEL_ELEVATOR_ID	0xFE

#define ANG_VEL_STD			0x300		// 0x300 = 108RPM = 18deg/s
#define ANG_VEL_114_RPM		0x3ff

#define CLOCKWISE_OFFSET	0x400
#define RPM_TO_DEGPS		6
#define DEGPS_TO_BITS		0.02346	
#define BITS_TO_DEGPS		42.625

/*************************************************************************
* Internal defines						                                     *
**************************************************************************/
#define PACKAGE_HEADER			0xFF
#define SEND_BYTE_INTERVAL		0.002	// unit = s


/*************************************************************************
* Dynamixel baud rate values			                                     *
**************************************************************************/
#define BdR1000000			1
#define BdR500000			3
#define BdR400000			4
#define BdR250000			7
#define BdR200000			9
#define BdR115200			16
#define BdR57600			34
#define BdR19200			103
#define BdR9600				207

/*************************************************************************
* Dynamixel instruction set			                                     *
**************************************************************************/
#define INSTR_PING			0x01
#define INSTR_READ			0x02
#define INSTR_WRITE			0x03
#define INSTR_REG_WRITE		0x04
#define INSTR_ACTION		0x05
#define INSTR_RESET			0x06
#define INSTR_REBOOT		0x08
#define INSTR_SYNC_WRITE	0x83
#define INSTR_BULK_READ		0x92


/*************************************************************************
* Dynamixel Registers				                                     *
**************************************************************************/
#define DYNAMIXEL_ID_REG			0x03
#define DYNAMIXEL_BAUD_RATE_REG		0x04
#define CW_ANGLE_LIMIT_REG			0x06
#define CCW_ANGLE_LIMIT_REG			0x08
#define LED_REG						0x19
#define CW_COMPLIANCE_MARGIN_REG	0x1A
#define CCW_COMPLIANCE_MARGIN_REG	0x1B
#define CW_COMPLIANCE_SLOPE_REG		0x1C
#define CCW_COMPLIANCE_SLOPE_REG	0x1D
#define GOAL_POSITION_REG			0x1E
#define MOVING_SPEED_REG			0x20
#define TORQUE_LIMIT_REG			0x22
#define PRESENT_POSITION_REG		0x24
#define PRESENT_SPEED_REG			0x26
#define PRESENT_LOAD_REG			0x28
#define LOCK_REG					0x2F

/*************************************************************************
* Internal global variables						                                     *
**************************************************************************/
//extern int dynamixel_uart_direction;
//
extern int uart0_filestream;

/*************************************************************************
* Functions Prototypes				                                     *
**************************************************************************/
/*
int Send_Instruction_Packet(Byte ID, Byte Length, Byte Instruction, Byte P1, Byte P2, Byte P3);
int Get_Status_Packet(Byte* ID, Byte* Length, Byte* Error, Byte* P1, Byte* P2, Byte* Checksum, Byte* Fail);

Byte Set_Parameter(Byte ID, Byte Length, Byte Address, int Parameter);
int Get_Parameters(Byte ID, Byte Address, Byte N);
*/

void UARTSetup();
void send5ByteMessage(char ID, char instruction, char param1, char param2, char param3);
void send4ByteMessage(char ID, char instruction, char param1, char param2);
void send3ByteMessage(char ID, char instruction);

void dynamixelSetup();

void dynamixelSetJointMode(char ID);
void dynamixelSetWheelMode(char ID);

void dynamixelWriteLed(char ID, char status);

void dynamixelSetGoalPositionDeg(char ID, double goal_position_deg);
void dynamixelSetGoalPosition(char ID, short int goal_position);
void dynamixelSetRelativeGoalPositionDeg(char ID, double goal_position_deg);
void dynamixelSetVelocity(char ID, short int velocity);

#endif /*DYNAMIXEL_H_*/


