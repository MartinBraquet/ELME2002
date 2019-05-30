#ifndef SERVER_H_
#define SERVER_H_

#include "../CtrlStruct_gr3.h"

#define ANDROID_PORT 5550

int SocketCreate();
int BindCreatedSocket(int Socket);
void init_socket(int *socket);
int android_app(CtrlStruct *cvs, double* V, double* W, unsigned char* buffer);

#endif

