/*
 * Serial.cpp
 *
 *  Created on: 15 de fev. de 2021
 *      Author: milton
 */

#include "Serial.h"

Serial::Serial(char const *ser, int baudRate) {
	// TODO Auto-generated constructor stub
	this->baudRate = baudRate;
	bzero(this->serDevice, sizeof(this->serDevice));
	strcpy(this->serDevice, ser);
	this->fd = -1;
	this->flowControl = 0; //0 None / 1 Enable RTS/CTS hardware flow control
	this->stopBits = 1;
	this->dataBits = 8;
	this->timeOutms = -1;
}

Serial::~Serial() {
	// TODO Auto-generated destructor stub
}

int Serial::openSerialPort(int timeOut) {
	this->fd = open(this->serDevice, O_RDWR | O_NOCTTY | O_SYNC);
	if (this->fd < 0) {
	    printf("Error %i from open: %s\n", errno, strerror(errno));
	    return -1;
	}

	if (this->fd == 2) { //No such file or directory
		printf("Error %i from open: %s\n", errno, strerror(errno));
		return -1;
	}

	if (this->fd == 13) { //Permission denied
		printf("Error %i from open: %s\n", errno, strerror(errno));
		return -1;
	}

	this->timeOutms = timeOut;

	configureTermios();

	return 0;
}

int Serial::readBytes(char *buff, int len){
	if(this->fd < 0) {
		printf("Error serial read, invalid file descriptor\n");
		return -1;
	}

	ssize_t n = read(this->fd, buff, len);

	//error handling
	if (n < 0) {
		printf("Error reading \n");
		return -1;
	}

	return n;
}

int Serial::writeBytes(const char *buff,int len){
	if (this->fd < 0) {
		printf("Error serial write, invalid file descriptor\n");
		return -1;
	}

	int writtenBytes = write(this->fd, buff, len);

	if (writtenBytes < 0) {
		printf("Error writting bytes on serial port");
		return -1;
	}

	return writtenBytes;
}

void Serial::closeSerialPort(){
	if (this->fd != -1) {
		int retVal = close(this->fd);
		if (retVal != 0){
			printf("Error clossing serial port\n");
		}
		this->fd = -1;
	}
}

int Serial::readLine(char *buff, int max){
	char c;
	int rcv, i = 0;

	while (max--) {

		rcv = readBytes(&c, 1);
		if (rcv > 0) {
			buff[i] = c;
			if (c == 0x0A) {
				buff[i + 1] = 0;
				return i;
			}
			i++;
		} else if (rcv == -1) {
			return rcv;
		}
	}
	return -2;
}

int Serial::getBaudRateValMacro(int baudRate) {
	int baudRateMacro;
	switch(baudRate) {

	case 0:
		baudRateMacro = B0;
		break;

	case 50:
		baudRateMacro = B50;
		break;

	case 75:
		baudRateMacro = B75;
		break;

	case 110:
		baudRateMacro = B110;
		break;

	case 134:
		baudRateMacro = B134;
		break;

	case B150:
		baudRateMacro = B150;
		break;

	case B200:
		baudRateMacro = B200;
		break;

	case 300:
		baudRateMacro = B300;
		break;

	case 600:
		baudRateMacro = 600;
		break;

	case 1200:
		baudRateMacro = B1200;
		break;

	case 1800:
		baudRateMacro = B1800;
		break;

	case 2400:
		baudRateMacro = B2400;
		break;

	case 4800:
		baudRateMacro = B4800;
		break;

	case 9600:
		baudRateMacro = B9600;
		break;

	case 19200:
		baudRateMacro = B19200;
		break;

	case 38400:
		baudRateMacro = B38400;
		break;

	case 57600:
		baudRateMacro = B57600;
		break;

	case 115200:
		baudRateMacro = B115200;
		break;

	case 230400:
		baudRateMacro = B230400;
		break;

	case 460800:
		baudRateMacro = B460800;
		break;

	default:
		printf("Invalid baudRate, setting 9600\n");
		baudRateMacro = B9600;
	}

	return baudRateMacro;
}

void Serial::configureTermios() {
	struct termios tty;


	//get termios
	// Read in existing settings, and handle any error
	// NOTE: This is important! POSIX states that the struct passed to tcsetattr()
	// must have been initialized with a call to tcgetattr() overwise behaviour
	// is undefined
	if(tcgetattr(this->fd, &tty) != 0) {
	    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	//set parity
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	//tty.c_cflag |= PARENB;  // Set parity bit, enabling parity

	//CSTOPB (Num. Stop Bits)
	if(this->stopBits == 2) {
		tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
	}else {
		tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	}


	//bis per byte
	tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
	switch (this->dataBits) {

	case 5:
		tty.c_cflag |= CS5; // 5 bits per byte
		break;

	case 6:
		tty.c_cflag |= CS6; // 6 bits per byte
		break;

	case 7:
		tty.c_cflag |= CS7; // 7 bits per byte
		break;

	default:
		tty.c_cflag |= CS8; // 8 bits per byte (most common)

	}

	//Flow Control (CRTSCTS)
	if (this->flowControl) {
		tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control
	}else {
		tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	}



	//Setting CLOCAL disables modem-specific signal lines such as carrier detect.
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	//Disabling cannonical mode
	tty.c_lflag &= ~ICANON;

	//echo
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo

	//Disable Signal Chars
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

	//Software Flow Control (IXOFF, IXON, IXANY)
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl

	//Disabling Special Handling Of Bytes On Receive
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	//Output Modes (c_oflag)
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

	//set timeout
    if(this->timeOutms == -1) {
        // Always wait for at least one byte, this could
        // block indefinitely
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 1;
    } else if(this->timeOutms == 0) {
        // Setting both to 0 will give a non-blocking read
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;
    } else if(this->timeOutms > 0) {
        tty.c_cc[VTIME] = (cc_t)(this->timeOutms/100);    // 0.5 seconds read timeout
        tty.c_cc[VMIN] = 0;
    }

    // Set in/out baud rate
    int baudRate = getBaudRateValMacro(this->baudRate);
    cfsetispeed(&tty, baudRate);
    cfsetospeed(&tty, baudRate);

    // Save tty settings, also checking for error
    if (tcsetattr(this->fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
}

char *Serial::getSerialPortName() {
	char *ret = &this->serDevice[0];
	return ret;
}

int Serial::getBaudRate() {
	return this->baudRate;
}
