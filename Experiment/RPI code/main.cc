#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "IO/SPI/SPI.hh"
#include "dynamixel/code_termios/MyDynamixel.h"

#define channel 0
#define clockSpi 500000
#define resetPin 25
#define PORT 5560

#define resetPin 25
#define led_experiment 22

/// Main structure
typedef struct LT24Struct
{
	int data_to_PI;
	int data_from_PI;

} LT24Struct;


int SocketCreate()
{
	int Socket;

	Socket = socket(AF_INET, SOCK_STREAM, 0);
	/*
	linger lin;
	lin.l_onoff = 0;
	lin.l_linger = 0;
	setsockopt(Socket, SOL_SOCKET, SO_LINGER, (const char *) &lin, sizeof(int));
*/
	int yes = 1;
	if (setsockopt(Socket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) < 0) {
	  printf("dead 1\n");
	}

	yes = 1;
	if (setsockopt(Socket, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(int)) < 0) {
	  printf("dead 2\n");
	}

	return Socket;
}
 
 
int BindCreatedSocket(int socket)
{
	int iRetval = -1;
	struct sockaddr_in address; 

	address.sin_family = AF_INET; /* Internet address family */
	address.sin_addr.s_addr = htonl(INADDR_ANY); 

	address.sin_port = htons(PORT); /* Port of the Client */
	iRetval = bind(socket, (struct sockaddr*) &address, sizeof(address));

    return iRetval;
}

void init_socket(int* Socket, int port) 
{
	int bindsocket;
	
	//Create socket
	*Socket = SocketCreate();
	if (*Socket == -1)
	{
		printf("Could not create socket\n");
	}
	else
	{
		printf("Socket created: %d\n", *Socket);
	}

	//Bind
	bindsocket = BindCreatedSocket(*Socket);
	if(bindsocket < 0)
	{
		printf("Bind failed\n");
	}
	else
	{
		printf("Bind done\n");
	}

	//Listen
	listen(*Socket, 1);
}

void *experiment_task(void *ptr) {
	printf("START: experiment_task\n");	

	//Set up the dynamixel
	dynamixelSetup();
	
	// Set the dynamixel in speed mode
	dynamixelSetWheelMode(DYNAMIXEL_EXPERIENCE_ID);
	
	//Start the motor to maximum speed
	dynamixelSetVelocity(DYNAMIXEL_EXPERIENCE_ID, 0x300);
	digitalWrite(led_experiment,HIGH);	
	sleep(32);
	
	// Stop the motor  but the LED is still on to show that 
	// the experiment was launched
	dynamixelSetVelocity(DYNAMIXEL_EXPERIENCE_ID, 0x0);

	printf("END: experiment_task\n");

	return 0;

}

void *SPI_LT24_task(void *ptr) {
	printf("START: SPI_LT24_task\n");

	LT24Struct *LT24struct = (LT24Struct*) ptr;

	//SETTING THE SPI INTERFACE
	printf("SPI setup: %d \n", wiringPiSPISetup(channel, clockSpi));
	SPI *spi;

	//Set the GPIO from the rapsberry PI 
	if(wiringPiSetup() == -1)
		printf("error : unable to set the GPI from the raspberry \n");
	
	pinMode(resetPin, OUTPUT);
	digitalWrite(resetPin, HIGH);
	delay(50);
	digitalWrite(resetPin, LOW);
	
	unsigned char read_buffer[100], write_buffer[100];
	
	while (1) {
		
		// Read the LT24 data
		read_buffer[0] = 0x00; read_buffer[1] = 0x00; read_buffer[2] = 0x00; read_buffer[3] = 0x00; read_buffer[4] = 0x00; 
		wiringPiSPIDataRW(channel, read_buffer, 5);
		LT24struct->data_to_PI = spi->frombytes(5, read_buffer);
		printf("From LT24 to Kraken: %X\n", LT24struct->data_to_PI);
		
		// Write data to LT24
		write_buffer[0] = 0x80; write_buffer[1] = 0x00; write_buffer[2] = 0x00; write_buffer[3] = 0x00; 
		write_buffer[4] = LT24struct->data_from_PI;
		wiringPiSPIDataRW(channel, write_buffer, 5);
		printf("From Kraken to LT24: %d\n", LT24struct->data_from_PI);
		
	}

	printf("END: SPI_LT24_task\n");

	return 0;

}

void *socket_PI_task(void *ptr) {
	printf("START: socket_PI_task\n");

	LT24Struct *LT24struct = (LT24Struct*) ptr;
	
	int client_message, server_message;
	int start_experiment = 1;
	char command[100];
	
	system("python show_score.py START");

	while (1) {
		
		int mysocket, sock, clientLen, read_size, sender;
		struct sockaddr_in server, client;
		
		client_message = 0;

		init_socket(&mysocket, PORT);

		//Accept an incoming connection

		printf("Waiting for incoming connections...\n");
		clientLen = sizeof(struct sockaddr_in);

		//accept connection from an incoming client
		sock = accept(mysocket, (struct sockaddr *) &client, (socklen_t*) &clientLen);

		printf("Accepted connection\n");

		//Receive a reply from the client
		if(recv(sock, &client_message, sizeof(client_message), 0) < 0) {
			printf("LT24: Reception failed\n");
			LT24struct->data_from_PI = -1;
		} else {
			printf("LT24: Client reply : %X\n", client_message);
			LT24struct->data_from_PI = client_message;
		}

		// Send some data
		server_message = LT24struct->data_to_PI;
		sender = send(sock, &server_message, sizeof(server_message), 0);

		close(sock);
		close(mysocket);
		
		printf("YOOoooooooooooooooooooooo\n", command);
		printf("%s\n", command);
		
		// Show score on LCD screen
		sprintf(command, "python show_score.py %d", LT24struct->data_from_PI);
		
			printf("%s\n", command);
		
		system(command);
		
		// Start experiment
		if (start_experiment) {
			start_experiment = 0;
			pthread_t experiment_thread;
			pthread_create(&experiment_thread, NULL, experiment_task, (void*) LT24struct);
		}
	}

	printf("END: socket_PI_task\n");

	return 0;

}

int main(int argc , char *argv[])
{
	
	//Initialisation 
	
	//GPIO board to interact with LED 
	if(wiringPiSetup() == -1)
	{
		printf("error : unable to set the GPI from the raspberry \n");
	}
		
	pinMode(resetPin, OUTPUT);
	pinMode(led_experiment ,OUTPUT);
	digitalWrite(led_experiment,LOW);
	
	//FPGA reset
	digitalWrite(resetPin, HIGH);
	delay(50);
	digitalWrite(resetPin, LOW);
	
	
	LT24Struct *LT24struct = (LT24Struct*) malloc(sizeof(LT24Struct));
	
	LT24struct->data_from_PI = 0;
	LT24struct->data_to_PI = 0;

	pthread_t SPI_LT24_thread;
	pthread_create(&SPI_LT24_thread, NULL, SPI_LT24_task, (void*) LT24struct);

	pthread_t socket_PI_thread;
	pthread_create(&socket_PI_thread, NULL, socket_PI_task, (void*) LT24struct);
	
	pthread_join(SPI_LT24_thread, NULL);
	pthread_join(socket_PI_thread, NULL);
	
	/*
	 * Just to check the motor code is working properly on it own, and it did !
	 * 
	 * pthread_t experiment_thread;
	pthread_create(&experiment_thread, NULL, experiment_task, (void*) LT24struct);
	pthread_join(experiment_thread, NULL);*/
	
	
	
	return 0;

}

