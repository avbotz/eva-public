/*
               _       _                   
 ___  ___ _ __(_) __ _| |  ___ _ __  _ __  
/ __|/ _ \ '__| |/ _` | | / __| '_ \| '_ \ 
\__ \  __/ |  | | (_| | || (__| |_) | |_) |
|___/\___|_|  |_|\__,_|_(_)___| .__/| .__/ 
                              |_|   |_|    
*/
/* Copyright (c) 2014 AVBotz
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ========================================================================== */
#include "serial.hpp"

Serial::Serial(const char* portName, State *s) :
	timpl(new ThreadImpl),
	state(s)
{
	// open the serial port
	serialPort = openPort(portName);
	// make sure the port was actually opened; exit if it was not
	assert(serialPort != -1);
}

Serial::~Serial()
{
	timpl->stop_requested = true;
	timpl->m_thread.join();
	close(serialPort);
	lprintf("Serial port closed.\n");
}

/*
 * Sets up the communication port with the mbed.
 * This code is basically copied from AVI
 */
int Serial::openPort(const char* portName)
{
	// open the first USB-serial converter
	// read-write, prevents file (terminal device) from becoming controlling device of eva, open the connection immediately
	int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
	// if opening the port failed
	if (fd == -1)
	{
		lprintf("Error: unable to open serial port %.", portName);
		return -1;
	}
	// get paramaters associated to fd
	tcgetattr(fd, &options);
	// baud rate = 115200
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	// ignore modem control lines, enable receiver
	options.c_cflag |= (CLOCAL | CREAD);
	// disable parity, 1 stop bit, remove any previous character size setting
	options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
	// 8 bit characters
	options.c_cflag |= CS8;
	// non-canonical input (dont wait for newline to send), disable echo, cant use erase character, dont forward beagleboard's signals to mbed
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	// disable implementation-defined output processing
	options.c_oflag &= ~OPOST;
	// dont wait for multiple characters to read, read every one
	options.c_cc[VMIN]  = 0;
	// no timeout for read
	options.c_cc[VTIME]  = 0;
	// set EVA as the receiver of SIGIO and SIGURG signals
	fcntl(fd, F_SETOWN, getpid());
	// read doesnt block
	fcntl(fd, F_SETFL, FNDELAY);
	// set the attributes effective immediately
	tcsetattr(fd, TCSANOW, &options);

	return (fd);
}



/*
 * Sends size characters of string str over the serial port
 */
int Serial::writeSerial(char* str, int size)
{
	return write(serialPort, str, size);
}
