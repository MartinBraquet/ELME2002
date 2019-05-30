#ifndef CLIENT_H_
#define CLIENT_H_

#include "../CtrlStruct_gr3.h"

#define LT24_PORT 5560

int SocketConnect(int Socket);
int SocketSend(int Socket, uint32_t* Rqst);
int SocketReceive(int Socket, int* Rsp);
void RW_data_LT24(CtrlStruct *cvs, int read_flag);

#endif

