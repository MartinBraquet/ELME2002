#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h> 
#include <unistd.h>
#include "client.h"
#include "server.h"
#include "../CtrlStruct_gr3.h"
#include "../strategy/strategy_gr3.h"
#include "../localization/opp_pos_gr3.h"
 
// Try to connect with server
int SocketConnect(int Socket)
{
	int iRetval = -1;
	struct sockaddr_in address; 

	address.sin_addr.s_addr = inet_addr("192.168.4.15"); //Local Host
	address.sin_family = AF_INET;
	address.sin_port = htons(LT24_PORT);

	iRetval = connect(Socket, (struct sockaddr *)&address, sizeof(struct sockaddr_in));

	return iRetval;
}
 

int SocketSend(int Socket, uint32_t* Rqst)
{
	int shortRetval = -1;
	
	shortRetval = send(Socket, Rqst , sizeof(Rqst), 0);
	//printf("data from client : %d\n\n",*Rqst);

	return shortRetval;
}
 
 
// Receive the data from the server 
int SocketReceive(int Socket, int* Rsp)
{
	int shortRetval = -1;

	shortRetval = recv(Socket, Rsp, sizeof(Rsp), 0);

	//printf("Response %s\n",Rsp);

	return shortRetval;
}


void RW_data_LT24(CtrlStruct *cvs, int read_flag)
{ 
	int Socket, read_size;
	struct sockaddr_in server;
	int error = 0;

	int server_reply = 0;
	uint32_t SendToServer = (uint32_t) cvs->outputs->score;

	// Create socket
	Socket = SocketCreate();
	if(Socket == -1) {
		printf("Could not create socket\n");
		error = 1;
	} else {
		printf("Socket is created\n");
	}

	// Connect to remote server
	if (SocketConnect(Socket) < 0) {
		error = 1;
		printf("Connect failed\n");
	} else {
		printf("Sucessfully connected with server\n");
	}
	
	if (error) {
		if (read_flag) {
			cvs->robot_team = 0;
			cvs->opp_pos->nb_opp = 1;
			cvs->main_strategy = 0;
		}
		return;
	}

	// Send data to the server
	SocketSend(Socket, &SendToServer);
	printf("Data from client : %d\n", SendToServer);


	if (read_flag) {
		// Received the data from the server
		SocketReceive(Socket, &server_reply);
		printf("Server response : %d\n", server_reply);
		server_reply -= 16384 + 3000;
		cvs->opp_pos->nb_opp  = (int) (server_reply / 100);
		server_reply -= cvs->opp_pos->nb_opp * 100;
		cvs->main_strategy  = (int) (server_reply / 10) - 1;
		server_reply -= (cvs->main_strategy+1) * 10;
		cvs->robot_team = (int) server_reply;
		printf("LT24 data: nb_opp: %d, robot_team: %d, main_strategy: %d\n", cvs->opp_pos->nb_opp, cvs->robot_team, cvs->main_strategy);
		
		if (cvs->robot_team == 0) {
			cvs->plus_or_minus = 1;
		} else {
			cvs->plus_or_minus = -1;
		}
		
		if (cvs->robot_team < 0 || cvs->robot_team > 1) {
			cvs->robot_team = 1;
		}
		if (cvs->opp_pos->nb_opp < 0 || cvs->opp_pos->nb_opp > 2) {
			cvs->opp_pos->nb_opp = 1;
		}
		if (cvs->main_strategy < 0 || cvs->main_strategy > 4) {
			cvs->main_strategy = 0;
		}
		
	}

	close(Socket);

	shutdown(Socket,0);

	shutdown(Socket,1);
	shutdown(Socket,2);
	   
}
