#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <termios.h>

// Data structs
#include "batteryData.h"
#include "actuatorData.h"
#include "powerRailData.h"
#include "touchPanelData.h"
#include "serialHelper.h"

// Xbee
#include "readXbee.h"


int main(void){

	struct timespec tim, tim2;
	tim.tv_sec  = 0;
	tim.tv_nsec = 50000000L;//0.05s

	char *port = "/dev/ttyUSB1";
	struct termios tty;
	int fd;

	const static uint8_t starting_mark = 0xAA;

	int recBytes = 5;
	int sendBytes = 96;

	uint8_t recBuffer[5] = {0};
	uint8_t buttonState[2] = {0};
	uint8_t sendBuffer[96] = {0};

	//touchPanelData data;
	//uint8_t n = packSerialDataPackage(data, &sendBuffer[0]);

	sendBuffer[0] = starting_mark;
	sendBuffer[1] = starting_mark;
	sendBuffer[2] = starting_mark;

	sendBuffer[3] = 90;

	sendBuffer[55] = getInt(100.456f);
	sendBuffer[56] = getDecimal(100.456f);

	sendBuffer[57] = getHigherByte((uint16_t)2000);
	sendBuffer[58] = getLowerByte((uint16_t)2000);

	sendBuffer[94] = buttonState[0];
	sendBuffer[95] = buttonState[1];

	// Initialize Xbee
	fd = xbee_init(port,&tty);

	if (fd == -1){exit(0);}

	while(1){

		int n = xbee_write(fd,sendBuffer,sendBytes);

		n = xbee_read (fd, &recBuffer[0], recBytes);

		if(n >0){
			sendBuffer[94] = recBuffer[3];
			sendBuffer[95] = recBuffer[4];
			parse_button_state(&recBuffer[3]);
		}

		nanosleep(&tim, NULL);

	}

}