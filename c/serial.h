/*
 * serial.h
 *
 *  Created on: 7 de fev. de 2021
 *      Author: milton seirra
 */

#ifndef SRC_SERIAL_SERIAL_H_
#define SRC_SERIAL_SERIAL_H_

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


typedef struct {
	int fd;
	char serDevice[50];
	int baudRate;
	int timeOutms;
	int flowControl;
	char stopBits;
	char dataBits;
}serial_obj_t;



/**
 * @brief Init config values for serial port object
 * @param serial Pointer to serial object
 * @param ser Serial device name
 * @param baudRate
 * @return void
**/
void Serial_setConfig(serial_obj_t *serial, char const *ser,int baudRate);

/**
 * @brief Open serial port
 * @param serial Pointer to serial object
 * @param timeOut Time Out for read
 * @return void
**/
void Serial_open(serial_obj_t *serial, int timeOut);

/**
 * @brief Reads bytes on serial port
 * @param serial Pointer to serial object
 * @param buff Pointer to byte buffer
 * @param len Size of bytes for read
 * @return void
**/
int Serial_read(serial_obj_t *serial,char *buff, int len);


/**
 * @brief Write bytes on serial port
 * @param serial Pointer to serial object
 * @param buff Pointer to byte buffer
 * @param len Size of bytes for read
 * @return void
**/
int Serial_write(serial_obj_t *serial,char *buff, int len);

/**
 * @brief Close serial port
 * @param serial Pointer to serial object
 * @return void
**/
void Serial_close(serial_obj_t *serial);


/**
 * @brief Reads one entire line from the serial port.
 * @param serial Pointer to serial object
 * @param buff Pointer to byte buffer
 * @param max This is the number of bytes to be read from the file
 * @return If error returns -1
**/
int Serial_readLine(serial_obj_t *serial, char *buff, int max);


#endif /* SRC_SERIAL_SERIAL_H_ */
