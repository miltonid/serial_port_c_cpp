/*
 * main.c
 *
 *  Created on: 7 de fev. de 2021
 *      Author: milton
 */
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>

#include "serial.h"


void *taskSend(void *arg){
	serial_obj_t *serial = (serial_obj_t *)arg;
	char msg[100];
	char cnt = 0;
	while (1) {
		usleep(1000000); //50ms
		cnt++;
		int cx = snprintf ( msg, 100, "counter = %d\n", cnt );
		printf("sending %s",msg);
		Serial_write(serial, msg, cx );
	}

}

int main()
{
	char buffer[100],c;
	int rcv;
	serial_obj_t serial1 ={0};
	Serial_setConfig(&serial1, "/dev/ttyACM0", 115200);

	memset(buffer, 0, sizeof(buffer));
	printf("loop\n");
	Serial_open(&serial1, 2000);
	pthread_t tid;
	pthread_create(&tid, NULL, taskSend, (void *)&serial1);
	while (1) {
		rcv = Serial_readLine(&serial1, buffer, sizeof(buffer));
		if (rcv > 0) {
			printf("rcv %s", buffer);
		}
	}

	return 0;
}


