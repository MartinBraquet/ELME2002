#include <stdio.h>
#include <string.h>    
#include <sys/socket.h>
#include <arpa/inet.h> 
#include <unistd.h>    
#include "../CtrlStruct_gr3.h"
#include "server.h"
 
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
	int length = sizeof(address); 


	address.sin_family = AF_INET; /* Internet address family */
	address.sin_addr.s_addr = htonl(INADDR_ANY); 

	/* Any incoming interface 
	convertit  un  entier  court  (short)
	hostshort  depuis  l'ordre des octets de l'hôte vers celui
	du réseau */

	address.sin_port = htons(ANDROID_PORT); /* Port of the Client */
	iRetval = bind(socket, (struct sockaddr*) &address, sizeof(address));

    return iRetval;
}

void init_socket(int* Socket) 
{
	int bindsocket;
	
	//Create socket
	*Socket = SocketCreate();
	if (*Socket == -1)
	{
		//printf("Could not create socket\n");
	}
	else
	{
		//printf("Socket created: %d\n", *Socket);
	}

	//Bind
	bindsocket = BindCreatedSocket(*Socket);
	if(bindsocket < 0)
	{
		//printf("Bind failed\n");
	}
	else
	{
		//printf("Bind done\n");
	}

	//Listen
	listen(*Socket, 1);
}
 
int android_app(CtrlStruct *cvs, double* V, double* W, unsigned char* buffer)
{
     int mysocket, sock, clientLen, read_size, sender;
     struct sockaddr_in server, client;
     uint32_t client_message;
     uint32_t message = (uint32_t) cvs->inputs->t;
     
     init_socket(&mysocket);

     //Accept an incoming connection
     
	 //printf("Waiting for incoming connections...\n");
	 clientLen = sizeof(struct sockaddr_in);
	 
	 //accept connection from an incoming client
	 sock = accept(mysocket, (struct sockaddr *) &client, (socklen_t*) &clientLen);
	 if (sock < 0)
	 {
		//printf("android: Accept failed\n");
	 }
	 else
	 {
		//printf("android: Connection accepted\n");
	 }
	 
	 //Receive a reply from the client
	 if(recv(sock, &client_message, sizeof(client_message), 0) < 0)
	 {
		//printf("android: reception failed\n");
	 }
	 else
	 {
		printf("android: Client reply : %X\n", client_message);
	 }
	 
	 
	 // Send some data
	 sender = send(sock, &message, sizeof(client_message), 0);
	 if(sender < 0)
	 {
		//printf("android: Send failed\n");
	 }
	 else
	 {
		 //printf("android: Send from server\n");
	 }
	 
	 close(sock);
	 close(mysocket);	
	 
	 
	 int flag = (client_message / 0x1000000) & 0x1;
	 
	 if (flag == 1) {
		 int strength = client_message & 0xFF;
		 int angle = (client_message / 0x100) & 0xFFFF;
		 *V = strength * cos(angle * M_PI / 180) / 10.0;
		 *W = strength * sin(angle * M_PI / 180) / 10.0;
		 
		 int test = (client_message / 0x10000) & 0xFF;
		 
		 cvs->outputs->pneumatic_commands = client_message / 0x2000000;
		 
		 //printf("%d %d %d\n", strength, angle, flag);
		 printf("%d\n", cvs->outputs->pneumatic_commands);
	 }
	 
	 return flag;
		   
}
