/*
 * Serial.h
 *
 *  Created on: 15 de fev. de 2021
 *      Author: milton
 */

#ifndef SRC_SERIAL_SERIAL_H_
#define SRC_SERIAL_SERIAL_H_

#include <stdio.h>   /* I/O Definitions                    */
#include <unistd.h>  /* Standard Symbolic Constants        */
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <string.h>  /* String Manipulation Definitions    */
#include <errno.h>   /* Error Code Definitions             */
#include <sys/ioctl.h>

#include <iostream>
#include <sys/time.h>

class Serial {
public:
	Serial(char const *ser, int baudRate);
	virtual ~Serial();
	void openSerialPort(int timeOut);
	int readBytes(char *buff, int len);
	int writeBytes(const char *buff, int len);
	void closeSerialPort();
	int readLine(char *buff, int max);

private:
	int fd;
	char serDevice[50];
	int baudRate;
	int timeOutms;
	int flowControl;
	char stopBits;
	char dataBits;
	int getBaudRateValMacro(int baudRate);
	void configureTermios();
};

#endif /* SRC_SERIAL_SERIAL_H_ */
