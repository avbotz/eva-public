/*
     _        _         _                 
 ___| |_ __ _| |_ ___  | |__  _ __  _ __  
/ __| __/ _` | __/ _ \ | '_ \| '_ \| '_ \ 
\__ \ || (_| | ||  __/_| | | | |_) | |_) |
|___/\__\__,_|\__\___(_)_| |_| .__/| .__/ 
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
#ifndef STATE_H__
#define STATE_H__

#include <sys/time.h>
#include <cxcore.h>

#include "vision.hpp"
#include "types.hpp"
#include "threadimpl.hpp"


#define STATE_PROP_NUM 11
#define IMAGE_NUM 2
#define EMERGENCIES_NUM 4

class State
{
public:
	State();
	~State();

	enum STATE_PROPERTY {
		currentHeading = 0,
		currentDepth,
		currentPower,
		desiredHeading,
		desiredDepth,
		desiredPower,
		dropperState,
		runState,
		missionState,
		lastTask,
		emergencyCode	// should be printed but not acted upon; mbed will handle safety

	};
	
	enum CAMERA_IMAGE {
		imageFront,
		imageDown
	};

	static long long getTimeStamp();
	static int getEquivalentAngle(int theta);
	static int getAngleDifference(int theta1, int theta2);
	
	static int getProperty(STATE_PROPERTY prop);
	static int setProperty(STATE_PROPERTY prop, int newValue);
	static bool checkProperty(STATE_PROPERTY prop, int value);
	
	static bool checkDesUpdated();
	// called by serial to notify state that des has been sent
	static void desHasBeenSent();
	
	static cv::Mat getImage(CAMERA_IMAGE direction);
	static void setImage(CAMERA_IMAGE direction, cv::Mat img);
	
	static const char* getEmergencyName();
	static const char* getEmergencyName(int emergencyValue);
	
private:
	static bool isDesUpdated;

	static int stateProps[STATE_PROP_NUM];
	static const int statePropRanges[STATE_PROP_NUM][2];

	enum {MIN, MAX};

	static Timer timer;

	static ThreadImpl* timpl;	// uses only the mutex for thread saftey
	
	static cv::Mat image[IMAGE_NUM];
	
	static const char* emergencyNames[EMERGENCIES_NUM];
};

#endif
