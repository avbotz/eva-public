/*
               _       _               _              _                   
 ___  ___ _ __(_) __ _| |    _ __ ___ | |__   ___  __| |  ___ _ __  _ __  
/ __|/ _ \ '__| |/ _` | |   | '_ ` _ \| '_ \ / _ \/ _` | / __| '_ \| '_ \ 
\__ \  __/ |  | | (_| | |   | | | | | | |_) |  __/ (_| || (__| |_) | |_) |
|___/\___|_|  |_|\__,_|_|___|_| |_| |_|_.__/ \___|\__,_(_)___| .__/| .__/ 
                       |_____|                               |_|   |_|    
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
#include "serial_mbed.hpp"

/*
 * Valid commands to send to AVNavControl.
 * Format is {[prefix], [source]}.
 * [source] is where Serial will get data to send to AVNavControl.
 */
const int SerialMbed::txList[][2] =
{
	{'h', State::desiredHeading},
	{'d', State::desiredDepth},
	{'p', State::desiredPower},
	{'r', State::dropperState},
};

/* 
 * Valid commands to receive from AVNavControl.
 * Format is {[prefix], [destination]}
 * [destination] is where Serial will put data from AVNavControl.
 */
const int SerialMbed::rxList[][2] =
{
	{'h', State::currentHeading},
	{'d', State::currentDepth},
	{'p', State::currentPower},
	{'e', State::emergencyCode},
};

SerialMbed::SerialMbed(const char* portName, State *s, int print_interval) : Serial(portName, s),
	resetRequested(false),
	mBufIndex(0),
	num_since_last_print(0)
	
{
	timpl->m_thread = std::thread(std::bind(&SerialMbed::mbedLoop, this));
	memset(messageBuf, 0, 20); // clear buffer
	interval = print_interval;
}

SerialMbed::~SerialMbed()
{

}

/*
 * Sends a command to the mbed in AVNav format
 * The prefix determines the value set
 * Add valid prefixes in Serial::commandList
 * The value is set to num
 */
int SerialMbed::writeCommand(char prefix, int num)
{
	// assemble the command
	char command[4];
	command[0] = prefix;
	command[1] = ((num>>6) & 0x3f) + 0x20;
	command[2] = (num & 0x3f) + 0x20;
	command[3] = '\n';
	
	// send it on the serial line and return whether it was successfully written
	return (writeSerial(command, 4));
}

int SerialMbed::reset()
{
	// request that mbedLoop() reset the mbed
	resetRequested = true;
	// wait for the request to complete
	while (resetRequested)
	{
		Timer::sleep(1);
	}
	return 0;
}

void SerialMbed::sendData()
{
	for (int i = 0; i < DATA_NUM_TX; i++)
	{
		writeCommand(txList[i][0], State::getProperty((State::STATE_PROPERTY)txList[i][1]));
	}
}

void SerialMbed::processData()
{
	int i = 0;
	int var, kill = -1;
	// the format for the message being processed:
	//	h__d__p__[l|k]e__
	// where [l|k] is l or k depending on the kill state
	while (messageBuf[i] != 0 && i < 20)
	{
		var = -1;
		for (int j = 0; j < DATA_NUM_RX; j++)
		{
			if (messageBuf[i] == rxList[j][0])
			{
				var = decodeAVNav(&messageBuf[i+1]);
				if (State::checkProperty((State::STATE_PROPERTY)rxList[j][1], var))
					State::setProperty((State::STATE_PROPERTY)rxList[j][1], var);
				break;
			}
			else if (messageBuf[i] == 'l')
			{
				kill = 0;
				break;
			}
			else if (messageBuf[i] == 'k')
			{
				kill = 1;
				break;
			}
		}
		if (var != -1)
		{
			i+=3;
			var = -1;
		}
		else
		{
			i++;
		}
	}
	
	// print current state to the screen
	num_since_last_print = (num_since_last_print + 1) % interval;
	if (num_since_last_print == 0)
	{
		lprintf("H:%    DH:%    D:%    DD:%    Pwr:%    K:%\n",
				State::getProperty(State::currentHeading),
				State::getProperty(State::desiredHeading),
				State::getProperty(State::currentDepth),
				State::getProperty(State::desiredDepth),
				State::getProperty(State::currentPower),
				kill);
	}
	
	// if there is an emergency, tell which kind
	if (State::getProperty(State::emergencyCode) != 0)
	{
		lprintf("* * * EMERGENCY: %! * * *\n", State::getEmergencyName());
	}

	if (kill != -1)
	{
		// makes sure it does not clear the 'stopped' flag, then appropriately sets the kill flag
		State::setProperty(State::runState, (State::getProperty(State::runState) & R_STOPPED) | !kill);
	}
}

/*
 * Converts encoded number from AVNav into an int
 *
 * AVNav data encoding:
 * Numbers are sent in 2 bytes, each with a value 0x20-0x5f, 32-95, or ' ' to '_'
 * The numbers are sent in base 64, with 0x20 being equal to 0 and 0x5f being equal to 63.
 * The first byte is the upper digit (most significant byte/big endian)
 * So the number 299 is sent as "$K" and is decoded as follows:
 *	$K => 0x24, 0x4B => 36, 75 => 4, 43 => 4*64 + 43 = 299
 * This keeps both bytes as readily readable ASCII characters that 
 *	are not the lower case letters used for commands
 */
inline int SerialMbed::decodeAVNav(char* data)
{
	return ((*data - 0x20)<<6) | (*(data+1) - 0x20);
}

void SerialMbed::mbedLoop()
{
	Timer t;
	t.resetTimer();
	while (!timpl->stop_requested)
	{
		// the loop runs
		int charsRead = read(serialPort, serialBuf, 256);
		if (charsRead)
		{
			for (int i = 0; i < charsRead; i++)
			{
				messageBuf[mBufIndex] = serialBuf[i];
				if (serialBuf[i] == '\n')
				{
					processData();
					// clear the buffer after reading data
					memset(messageBuf, 0, 20);
					mBufIndex = 0;
				}
				else
				{
					mBufIndex = (mBufIndex + 1) % 20;
				}
			}
		}
		// if new commands need to be sent then do it
		if (State::checkDesUpdated()) {
			std::lock_guard<std::mutex> lock(timpl->m_mutex);
			sendData();
			State::desHasBeenSent();
		}
		if (resetRequested) {
			std::lock_guard<std::mutex> lock(timpl->m_mutex);
			// the mbed will reset and load the most recently modified program
			// from flash if it receives a serial break character.
			int break_status = tcsendbreak(serialPort, 0);
			assert(break_status == 0);
			// wait for the mbed to reset and flash itself
			Timer::sleep(500);
			// clear the flag to indicate that the mbed was reset
			resetRequested = false;
		}
	}
}
