/*************************************************************************
* ELME 2002 : Project in Mechatronics                                  *
* Copyright (c) 2018 UCL							                     *
**************************************************************************
* MyDynamixel.c            											     *
* 	This file contains several functions used to communicate with the 	 *
* 	Dynamixels. They've been tested on AX-12 with a baudrate of 57600    *
* 	bps with a RaspberryPi 			                                     *
**************************************************************************
* Written by Arthur Alves Tasca						 					 *
* Version : 1.00 - March 2019   	                                     *
**************************************************************************/
#include "Dynamixel.h"


/****************************************************/
/****************************************************/
/*********		Dynamixel functions			*********/
/****************************************************/
/****************************************************/

///////////////////////////////////////////////////////////
//	dynamixelSetup()
///////////////////////////////////////////////////////////

void dynamixelSetup() {

	wiringPiSetup();

	pinMode(UART_DIRECTION_PORT, OUTPUT);
	digitalWrite(UART_DIRECTION_PORT, HIGH);

	UARTSetup();
}

///////////////////////////////////////////////////////////
//	dynamixelReboot(char ID)
//	INPUTS:
//	ID: dynamixel ID
///////////////////////////////////////////////////////////

void dynamixelReboot(char ID) {

	send3ByteMessage(ID, INSTR_REBOOT);

	sleep(0.5);
}
///////////////////////////////////////////////////////////
//	dynamixxelWrieLed(int ID, char status)
//	ID: dynamixel ID
//	status:	ON  -> 1
//			OFF -> 0
///////////////////////////////////////////////////////////

void dynamixelWriteLed(char ID, char status){
	printf("trying to turn on/off led...\n");

	send4ByteMessage(ID, INSTR_WRITE, LED_REG, status);

	sleep(0.1);

	//close(uart0_filestream);

}

///////////////////////////////////////////////////////////
//	dynamixelSetRelativeGoalPositionDeg()
// 
//	INPUTS
//	ID: dynamixel ID
//	goal_position_deg: goal position in degrees 
//						=>0 --> counter-clockwise
//						 <0 -- > clockwise
//   obs: this function will later reset the reference
//			start position of the dynamixel.
//					
//	NOT TESTED YET
///////////////////////////////////////////////////////////

void dynamixelSetRelativeGoalPositionDeg(char ID, double goal_position_deg) {
	
	int time_actuation_s;
	int velocity;

	time_actuation_s = abs(goal_position_deg) * RPM_TO_DEGPS / (ANG_VEL_STD * BITS_TO_DEGPS);
	if (goal_position_deg < 0)
		velocity = ANG_VEL_STD;
	else
		velocity = ANG_VEL_STD + CLOCKWISE_OFFSET;

	dynamixelSetWheelMode(ID);

	// starts the motor
	dynamixelSetVelocity(char ID, velocity);
	
	// sleeps so that it runs for the time proportional to the displacement
	sleep(time_actuation_s);

	// resets the motor
	dynamixelSetVelocity(char ID, 0);

	sleep(0.01);
}


///////////////////////////////////////////////////////////
//	dynamixelSetGoalPositionDeg()
// 
//	INPUTS
//	ID: dynamixel ID
//	goal_position_deg: goal position in degrees 
//					
//	NOT WORKING YET
///////////////////////////////////////////////////////////

void dynamixelSetGoalPositionDeg (char ID, double goal_position_deg) {
	
	// if operating under normal joint mode
	if(goal_position_deg >= 0 && goal_position_deg <= 0){

		dynamixelSetJointMode(ID);
		sleep(0.1);

		//converting from deg to unity at memory (1 = 0.29 deg)
		short int goal_position = (short int)(goal_position_deg * 3.448);
		///printf("trying to write: %02hhX %02hhX\n", (char)goal_position, (char)(goal_position >> 8));

		send5ByteMessage(ID, INSTR_WRITE, GOAL_POSITION_REG, (char)goal_position, (char)(goal_position >> 8));
	}

	sleep(0.1);
}

///////////////////////////////////////////////////////////
//	dynamixelSetGoalPosition()
//
//	INPUTS
//	ID: dynamixel ID
//	goal_position_deg: goal position in native unitr (0.29deg)
//					
///////////////////////////////////////////////////////////

void dynamixelSetGoalPosition(char ID, short int goal_position) {

	if (goal_position >= 0 && goal_position <= 0x3ff) {
		//converting from deg to unity at memory (1 = 0.29 deg)
		printf("trying to write: %02hhX %02hhX\n", (char)goal_position, (char)(goal_position >> 8));

		send5ByteMessage(ID, INSTR_WRITE, GOAL_POSITION_REG, (char)goal_position, (char)(goal_position >> 8));
		//	send5ByteMessage(ID, INSTR_WRITE, MOVING_SPEED_REG, (char) goal_position, (char) (goal_position >> 8) );
	}
	sleep(0.1);

}

///////////////////////////////////////////////////////////
//	dynamixelSetVelocity()
// 
//	INPUTS
//	ID: dynamixel ID
//	goal_position_deg: goal position in native unitr (0.29deg)
//					
///////////////////////////////////////////////////////////

void dynamixelSetVelocity(char ID, short int velocity) {

	printf("trying to write: %02hhX %02hhX\n", (char) velocity, (char) ( velocity>>8 ) );
	
	send5ByteMessage(ID, INSTR_WRITE, MOVING_SPEED_REG, (char) velocity, (char) (velocity >> 8) );

	sleep(0.1);

}

///////////////////////////////////////////////////////////
//	dynamixelSetJointMode()
//		joint mode is used to control position of the 
//		dynamixel. Sets the angle limits between 0 and  
//		1024
///////////////////////////////////////////////////////////
void dynamixelSetJointMode(char ID){

	// sets CW angle limit to 0x0000
	send5ByteMessage(ID, INSTR_WRITE, CW_ANGLE_LIMIT_REG, 0x00, 0x00 );

	sleep(0.5);

	// sets CCW angle limit to 0x3FF
	send5ByteMessage(ID, INSTR_WRITE, CCW_ANGLE_LIMIT_REG, 0xFF, 0x03 );

	sleep(0.5);

}

///////////////////////////////////////////////////////////
//	dynamixelSetJointMode()
//		joint mode is used to control position of the 
//		dynamixel. Sets the angle limits between 0 and  
//		1024
///////////////////////////////////////////////////////////
void dynamixelSetWheelMode(char ID){

	sleep(0.2);

	// sets CW angle limit to 0x0000
	send5ByteMessage(ID, INSTR_WRITE, CW_ANGLE_LIMIT_REG, 0x00, 0x00 );

	sleep(0.2);

	// sets CCW angle limit to 0x000
	send5ByteMessage(ID, INSTR_WRITE, CCW_ANGLE_LIMIT_REG, 0x00, 0x00 );

	sleep(0.2);

}
/****************************************************/
/****************************************************/
/*********		UART communication			*********/
/****************************************************/
/****************************************************/

///////////////////////////////////////////////////////////
//	UARTSetup()
///////////////////////////////////////////////////////////

void UARTSetup(){
//-------------------------
	//----- SETUP USART 0 -----
	//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
	uart0_filestream = -1;
	
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	uart0_filestream = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B57600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);

	close(uart0_filestream);
}

///////////////////////////////////////////////////////////
//	UARTSetup()
///////////////////////////////////////////////////////////

void send5ByteMessage(char ID, char instruction, char param1, char param2, char param3) {

	digitalWrite(UART_DIRECTION_PORT, HIGH);


	//----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;

	char length = 5;

	char checksum = (char)~(ID + length + instruction + param1 + param2 + param3);

	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = PACKAGE_HEADER;
	*p_tx_buffer++ = PACKAGE_HEADER;
	*p_tx_buffer++ = ID;
	*p_tx_buffer++ = length;
	*p_tx_buffer++ = instruction;
	*p_tx_buffer++ = param1;
	*p_tx_buffer++ = param2;
	*p_tx_buffer++ = param3;
	*p_tx_buffer++ = checksum;

	uart0_filestream = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
		}
	}

	close(uart0_filestream);
}

///////////////////////////////////////////////////////////
//	UARTSetup()
///////////////////////////////////////////////////////////


void send4ByteMessage(char ID, char instruction, char param1, char param2){

	digitalWrite(UART_DIRECTION_PORT, HIGH);


	//----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;

	char length = 4;

	char checksum = (char) ~(ID + length + instruction + param1 + param2);

	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = PACKAGE_HEADER;
	*p_tx_buffer++ = PACKAGE_HEADER;
	*p_tx_buffer++ = ID;
	*p_tx_buffer++ = length;
	*p_tx_buffer++ = instruction;
	*p_tx_buffer++ = param1;
	*p_tx_buffer++ = param2;
	*p_tx_buffer++ = checksum;
	
	uart0_filestream = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
		}
	}

	close(uart0_filestream);

}


///////////////////////////////////////////////////////////
//	UARTSetup()
///////////////////////////////////////////////////////////

void send3ByteMessage(char ID, char instruction){

	digitalWrite(UART_DIRECTION_PORT, HIGH);


	//----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;

	char length = 2;

	char checksum = (char) ~(ID + length + instruction);

	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = PACKAGE_HEADER;
	*p_tx_buffer++ = PACKAGE_HEADER;
	*p_tx_buffer++ = ID;
	*p_tx_buffer++ = length;
	*p_tx_buffer++ = instruction;
	*p_tx_buffer++ = checksum;
	
	uart0_filestream = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
		}
	}

	close(uart0_filestream);

}


///////////////////////////////////////////////////////////
//	UARTSetup()
///////////////////////////////////////////////////////////