#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define channel 0

int main(void)
{	
	int fd = wiringPiSPISetup(channel, 500000);
    	unsigned char buffer[100];

	printf("Initialisation %d \n", fd);
	
	// Aller lire dans un registre 
	buffer[0] = 0x02; 
	buffer[1] = 0x00;
	buffer[2] = 0x00;
	buffer[3] = 0x00;
	buffer[4] = 0x00;
	
	printf("Send data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
   	wiringPiSPIDataRW(channel, buffer, 5);
	printf("Received data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);

	// Aller écrire dans un registre
	
	buffer[0] = 0x80;
	buffer[1] = 0x00;
	buffer[2] = 0x00;
	buffer[3] = 0x00;
	buffer[4] = 0x01; //If 0 the first led should be off, if 1 the first led should be on
	
	printf("Send data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
   	wiringPiSPIDataRW(channel, buffer, 5);
	printf("Received data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
}
