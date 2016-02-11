#include "CAN.hh"
#include <cstdio>

CAN::CAN(uint32_t baudrate)
{
	this->baudrate = baudrate;
	this->spi2can = new SPI_CAN(1, 500e3);

}

CAN::~CAN()
{
	delete spi2can;
}

void CAN::configure()
{
	
    uint8_t canctrl, canstat, status, rxb0ctrl, rxb1ctrl, pinctrl, intere, interf;
	
	canctrl = 0b100 << 5 | 0b10000; // config mode
	spi2can->writeReg(CANCTRL, 1, &canctrl);

	canstat = spi2can->getBitsField(CANSTAT, 5, 3);
	printf("Config mode: 0x%x \n", canstat);

	status = spi2can->readStatus();
	printf("Device status: 0x%x \n", status);

	// Set baud rate
	// Due to the 16MHz clock, 8tq allows to achieve 250kbaud exactly
	spi2can->bitModify(CNF2, 0b00111000, 2 << 3);
	double tq = 1.0/8/CAN_BD; //Desired time quanta
	int brp = floor(tq*FOSC/2 -1); //Corresponding baudrate prescaler
	spi2can->bitModify(CNF1, 0b111111, brp);

	// Accept any message on rxb0 and rollover on rxb1 if full
	rxb0ctrl = 0b11 << 5 | 1 << 2;
	spi2can->writeReg(RXB_CTRL[0], 1, &rxb0ctrl);

	rxb1ctrl = 0b11 << 5;
	spi2can->writeReg(RXB_CTRL[1], 1, &rxb1ctrl);

	// Pin mode
	pinctrl = 0b1111;
	spi2can->writeReg(BFPCTRL, 1, &pinctrl);

	// Interrupts
	intere = 0xFF ;
	spi2can->writeReg(CANINTE, 1, &intere);
    	interf = 0x0;
	spi2can->writeReg(CANINTF, 1, &interf);

	// Stop transmission
	spi2can->bitModify(TXB_CTRL[0], 0b1011, 0x0);
	spi2can->bitModify(TXB_CTRL[1], 0b1011, 0x0);

	canctrl = 0b000 << 5 | 0b00000;
	spi2can->writeReg(CANCTRL, 1, &canctrl);
	
    	canstat = spi2can->getBitsField(CANSTAT, 5, 3);

	while (canstat != canctrl >> 5){
    	canstat = spi2can->getBitsField(CANSTAT, 5, 3);
	}
    
    	printf("Mode changed successfully: 0x%x \n", canstat);
}

int CAN::sendMessage(CANMessage *msg, uint8_t buf_nb)
{
	int txreq, flag_sent;
	uint8_t sidh, sidl, dlc, buf_ctrl, status;
	
	spi2can->bitModify(CANINTF, 0b1100, 0x0); //Clear transmission interrupts;

	//Check if transmit buffer ready
	if(spi2can->getBitsField(TXB_CTRL[buf_nb], 3, 1)){
        	// printf("Buffer not ready \n");
        	return -1;
	}
    
	//Load the message
	sidh = msg->sid >> 3;
	sidl = (msg->sid % (0b1<<4)) << 5;
	dlc = msg->rtr << 6 | msg->len;

	spi2can->writeReg(TXB_SIDH[buf_nb], 1, &sidh);
	spi2can->writeReg(TXB_SIDL[buf_nb], 1, &sidl);
	spi2can->writeReg(TXB_DLC[buf_nb], 1, &dlc);

	spi2can->loadTXBuf(buf_nb, msg->len, msg->data);
	uint8_t buf[10];

	msg->priority = fmin(msg->priority,3);

	//Launch transmission
	buf_ctrl = 1 << 3| msg->priority; //Ask transmission with priority
   	spi2can->bitModify(TXB_CTRL[buf_nb], 0b1011, buf_ctrl);

	/*
    	while(spi2can->getBitsField(TXB_CTRL[buf_nb], 3, 1)){ // Wait for completion
		spi2can->readReg(EFLG, 1, &status);
		//printf("Device errors: 0x%x \n", status);
	}*/

	//status = spi2can->getBitsField(TXB_CTRL[buf_nb], 4, 4);
	//printf("Message sent with status: 0x%x \n", status);

	
}

int CAN::readMessage(CANMessage *msg, uint8_t buf_nb)
{
	uint8_t sidd[2];

	if (!spi2can->getBitsField(CANINTF, buf_nb, 1)){
        	return -1;
	}
    
	spi2can->readReg(RXB_SIDH[buf_nb], 2, sidd);
	msg->sid = sidd[0]*8 | sidd[1] >> 5;
	msg->rtr = (sidd[1] >> 4)%2;
	msg->len = spi2can->getBitsField(RXB_DLC[buf_nb],0,3);

	spi2can->readRXBuf(buf_nb, msg->len, msg->data);

	//Clear interrupt
	spi2can->bitModify(CANINTF, 0b1+buf_nb, 0);

	return 1;
}

void CAN::check_receive()
{
	uint8_t status, irx0, irx1;
	spi2can->readReg(CANINTF, 1, &status);
	irx0 = (status     )%2;
	irx1 = (status >> 1)%2;
	
	CANMessage msg;

	if(irx0){
		readMessage(&msg, 0);
		decipher_msg(&msg);
	}
	if(irx1){
		readMessage(&msg, 1);
		decipher_msg(&msg); 
	}	
}

void CAN::decipher_msg(CANMessage *msg)
{
	
	if(msg->sid == 0x301 || msg->sid == 0x401){
		// printf("Acknowledge \n");
	}else if(msg->sid == 0x302 || msg->sid == 0x402){
		printf("data: 0x%x 0x%x 0x%x \n",msg->data[0], msg->data[1], msg->data[2]); 

		//int n = msg->data[1];
		//while (n) {
		//    if (n & 1)
		//        printf("1");
		//    else
		//        printf("0");
		//
		//    n >>= 1;
		//}
		//printf("\n");
		sw[SWL] = (msg->data[0] >> 3)%2 == 0;
		sw[SWR] = (msg->data[0] >> 1)%2 == 0;
		printf("Micro-switches: %d %d ! \n", sw[SWL], sw[SWR]);
		
	}else if(msg->sid == 0x300 || msg->sid == 0x400){
		printf("WakeUp message \n");
	}else{
		
		printf("Undeciphered msg for 0x%x \n", msg->sid);
		for(int i=0; i<msg->len; ++i)
		{
			printf("0x%x ", msg->data[i]);
		}
	}	

}

void CAN::ctrl_motor(int state)
{
	CANMessage msg;

	msg.rtr = 0;
	msg.priority = 3;
	msg.data[0] = 0x1E;
	msg.data[1] = 0x30;
	msg.data[2] = state? 0x00: 0xFF;
	msg.len = 3;
	msg.sid = CAN_MOT;
    	
	sendMessage(&msg, 0);

	msg.sid = CAN_TOW;
	sendMessage(&msg, 1);
}

void CAN::push_PropDC(int dcG, int dcD)
{
	
	uint8_t dcGc = 128*dcG/100.0 +128;
	uint8_t dcDc = 128*dcD/100.0 +128;

	// printf("G: %d D: %d \n", dcGc, dcDc);

	CANMessage msg;

	msg.rtr = 0;
	msg.priority = 3;
	msg.len = 3;
	msg.sid = CAN_MOT;

	msg.data[0] = 0x25;
	msg.data[1] = 0xFF;
	msg.data[2] = dcGc >> 2;
    	
	sendMessage(&msg, 0);

	msg.data[0] = 0x26;
	msg.data[1] = 0xFF;
	msg.data[2] = dcDc >> 2;

	sendMessage(&msg, 1);
	
}

void CAN::push_TowDC(int dc)
{
    	uint8_t dcc = 128*dc/100.0 +128;

	CANMessage msg;

	msg.rtr = 0;
	msg.priority = 3;
	msg.len = 3;
	msg.sid = CAN_TOW;

	msg.data[0] = 0x25;
	msg.data[1] = 0xFF;
	msg.data[2] = dcc >> 2;
    	
	sendMessage(&msg, 0);
}

void CAN::ctrl_led(int state)
{
    CANMessage msg;

    msg.len = 3;
    msg.rtr = 0;
    msg.priority = 3;
    msg.data[0] = 0x1E;
    msg.data[1] = 0x40;
    msg.data[2] = state? 0x40 : 0x0;

    msg.sid = CAN_MOT;
    sendMessage(&msg, 0);

    msg.sid = CAN_TOW;
    sendMessage(&msg, 1);

}