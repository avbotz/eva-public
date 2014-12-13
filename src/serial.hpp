/*
               _       _   _                 
 ___  ___ _ __(_) __ _| | | |__  _ __  _ __  
/ __|/ _ \ '__| |/ _` | | | '_ \| '_ \| '_ \ 
\__ \  __/ |  | | (_| | |_| | | | |_) | |_) |
|___/\___|_|  |_|\__,_|_(_)_| |_| .__/| .__/ 
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
#ifndef SERIAL_H__
#define SERIAL_H__

#include <assert.h>
#include <termios.h>
#include <fcntl.h>
#include <libconfig.h++>
#include <string.h>
#include <unistd.h>

#include "threadimpl.hpp"
#include "timer.hpp"
#include "state.hpp"

#define COMMAND_NUM 3

class Serial
{
public:
	Serial(const char* portName, State *s);
	~Serial();
	
	int writeSerial(char* str, int size);

protected:
	int openPort(const char* portName);
	
	ThreadImpl* timpl;

	struct termios options;
	int serialPort;

	int minDepth;
	
	State* state;
};


#endif
