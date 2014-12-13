/*
               _       _               _              _   _                 
 ___  ___ _ __(_) __ _| |    _ __ ___ | |__   ___  __| | | |__  _ __  _ __  
/ __|/ _ \ '__| |/ _` | |   | '_ ` _ \| '_ \ / _ \/ _` | | '_ \| '_ \| '_ \ 
\__ \  __/ |  | | (_| | |   | | | | | | |_) |  __/ (_| |_| | | | |_) | |_) |
|___/\___|_|  |_|\__,_|_|___|_| |_| |_|_.__/ \___|\__,_(_)_| |_| .__/| .__/ 
                       |_____|                                 |_|   |_|    
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
#ifndef S_MBED_H__
#define S_MBED_H__

#include "serial.hpp"

#define DATA_NUM_TX (sizeof(txList) / sizeof(txList[0]))
#define DATA_NUM_RX (sizeof(rxList) / sizeof(rxList[0]))

class SerialMbed : public Serial
{
public:
	SerialMbed(const char* portName, State *s, int print_interval);
	~SerialMbed();

	int reset();
	int writeCommand(char prefix, int num);

private:
	inline int decodeAVNav(char* data);
	void mbedLoop();
	void processData();
	void sendData();


	volatile bool resetRequested;

	char serialBuf[256];
	char messageBuf[20];
	int mBufIndex;
	
	int interval;
	int num_since_last_print;
	
	static const int txList[][2];
	static const int rxList[][2];
};


#endif
