/*
     _        _                         
 ___| |_ __ _| |_ ___   ___ _ __  _ __  
/ __| __/ _` | __/ _ \ / __| '_ \| '_ \ 
\__ \ || (_| | ||  __/| (__| |_) | |_) |
|___/\__\__,_|\__\___(_)___| .__/| .__/ 
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
#include "state.hpp"


const int State::statePropRanges[STATE_PROP_NUM][2] =
{
	{0, 359},					// current heading
	{0, 200},					// current depth
	{0, 200},					// current power
	{0, 359},					// desired heading
	{0, 200},					// desired depth
	{0, 200},					// desired power
	{0,   2},					// dropper state	
	{R_ALIVE, R_STOPPED | R_KILLED},		// run state
	{T_GATE, T_PARTY},				// current task
	{T_NUL, T_PARTY},				// last task completed
	{0, EMERGENCIES_NUM - 1}			// emergency code
};


// so that we can only send the desired when we update it and need to
bool State::isDesUpdated;

// state properties
int State::stateProps[STATE_PROP_NUM];

// the images which the tasks can obtain
cv::Mat State::image[IMAGE_NUM];

Timer State::timer;

const char* State::emergencyNames[] = {
	"No emergency",
	"Leaking",
	"Battery voltage low",
	"Battery voltage high",
};

// state requires the locks
ThreadImpl* State::timpl = new ThreadImpl;

State::State()
{
	timer.resetTimer();

	// nothing to send to WALL-E
	isDesUpdated = true;

	// reset the state
	stateProps[currentHeading]	= 0;
	stateProps[currentDepth]	= 0;
	stateProps[currentPower]	= 100;
	stateProps[desiredHeading]	= 0;
	stateProps[desiredDepth]	= 0;
	stateProps[desiredPower]	= 100;
	stateProps[dropperState]	= 0;
	stateProps[runState]		= R_KILLED;
	stateProps[missionState]	= T_GATE;
	stateProps[lastTask]		= T_NUL;
	stateProps[emergencyCode]	= 0;
}

State::~State()
{
}


long long State::getTimeStamp()
{
	return timer.getCurrentTime();
}

int State::getProperty(STATE_PROPERTY prop)
{
	if (prop < 0 || prop >= STATE_PROP_NUM)
		return -1;
	
	std::lock_guard<std::mutex> lock(timpl->m_mutex);
	return stateProps[prop];
}


// return -1 if bad index, otherwise what the property was actually set to
int State::setProperty(STATE_PROPERTY prop, int newValue)
{
	if (prop < 0 || prop >= STATE_PROP_NUM)
		return -1;
	if (stateProps[prop] == newValue)
		return newValue;
	
	// clamp the values
	if (newValue < statePropRanges[prop][MIN])
		newValue = statePropRanges[prop][MIN];
	else if (newValue > statePropRanges[prop][MAX])
		newValue = statePropRanges[prop][MAX];

	std::lock_guard<std::mutex> lock(timpl->m_mutex);
	
	// if eva needs to send new commands to the mbed
	if (prop == desiredHeading	|| 
		prop == desiredDepth	|| 
		prop == desiredPower	||
		prop == dropperState) {
			isDesUpdated = true;
	}

	stateProps[prop] = newValue;
	return newValue;
}

// returns true if the value is valid for a given property
// returns false if it isn't
bool State::checkProperty(STATE_PROPERTY prop, int value)
{
	return (value >= statePropRanges[prop][MIN] &&
			value <= statePropRanges[prop][MAX]);
}

bool State::checkDesUpdated()
{
	std::lock_guard<std::mutex> lock(timpl->m_mutex);
	return isDesUpdated;
}

int State::getEquivalentAngle(int theta)
{
	if (theta < 0) {
		return theta+360;
	}
	if (theta >= 360) {
		return theta-360;
	}
	return theta;
}

// returns + if theta1 is CW of theta2
// returns - if theta1 is CCW of theta2
int State::getAngleDifference(int theta1, int theta2)
{
	int dif = theta1 - theta2;
	if (-180 < dif && dif < 180)
		return dif;
		
	if (dif > 180)
		return dif - 360;
		
	if (dif < -180)
		return dif + 360;
		
	// dif is 180 or -180
	return 180;
}


cv::Mat State::getImage(CAMERA_IMAGE direction)
{
	if (direction < 0 || direction >= IMAGE_NUM)
		return cv::Mat();
	std::lock_guard<std::mutex> lock(timpl->m_mutex);
	return image[direction];
}
void State::setImage(CAMERA_IMAGE direction, cv::Mat img)
{
	if (direction < 0 || direction >= IMAGE_NUM)
		return;
	std::lock_guard<std::mutex> lock(timpl->m_mutex);
	image[direction] = img;
}

void State::desHasBeenSent()
{
	std::lock_guard<std::mutex> lock(timpl->m_mutex);
	isDesUpdated = false;
}

const char* State::getEmergencyName()
{
	return getEmergencyName(stateProps[emergencyCode]);
}

const char* State::getEmergencyName(int emergencyType)
{
	if (State::checkProperty(emergencyCode, emergencyType))
	{
		return emergencyNames[emergencyType];
	}
	else
	{
		return nullptr;
	}
}
