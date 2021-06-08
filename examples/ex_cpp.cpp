/*
 * main.cpp
 *
 *  Created on: 15 de fev. de 2021
 *      Author: milton
 */
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include "Serial.h"


void *taskSend(void *arg) {
	Serial *ser = (Serial *)arg;
	char msg[100];
	char cnt = 0;
	while (1) {
		usleep(1000000); //50ms
		cnt++;
		int cx = snprintf ( msg, 100, "counter = %d\n", cnt );
		printf("sending %s",msg);
		ser->writeBytes(msg, cx);
	}

}

int main(int argc, char *argv[]) {
	char buffer[100];
	int rcv;
	memset(buffer, 0, sizeof(buffer));
	Serial *ser = new Serial("/dev/ttyUSB0", 115200);
	ser->openSerialPort(2000);
	pthread_t tid;
	pthread_create(&tid, NULL, taskSend, (void *)ser);

	while (1) {
		rcv = ser->readLine(buffer, sizeof(buffer));
		if (rcv > 0) {
			printf("rcv %s", buffer);
		}
	}

}

