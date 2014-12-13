/*
             _                         
  __ _  __ _| |_ ___   ___ _ __  _ __  
 / _` |/ _` | __/ _ \ / __| '_ \| '_ \ 
| (_| | (_| | ||  __/| (__| |_) | |_) |
 \__, |\__,_|\__\___(_)___| .__/| .__/ 
 |___/                    |_|   |_|    
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
#include "gate.hpp"

Gate::Gate(libconfig::Config *c) : VisionTask(c, CAM_FRONT),
	grayImg(pic_height, pic_width, CV_8UC1)
{
	// read configuration-file data about how we should run gate into variables.
	conf->lookupValue("gate.time", gateTime);
	conf->lookupValue("gate.depth", gateDepth);
	conf->lookupValue("gate.power", gatePower);
	conf->lookupValue("gate.min_total_intensity", minTotalIntensity);
	conf->lookupValue("gate.min_pixel_intensity", minPixelIntensity);
	conf->lookupValue("gate.vision_time",visionTime);
	conf->lookupValue("gate.initial_pause_time", pauseTime);

	labelStage("Check for Gate");
	labelStage("Move Forward");
	labelStage("Finished");
}

Gate::~Gate()
{
	
}

void Gate::action()
{
	switch (stage)
	{
	case 0:
		// maintain initial heading because the sub might turn before moving forward.
		State::setProperty(State::desiredHeading, 0);
		
		// set current depth based on the configuration file.
		State::setProperty(State::desiredDepth, gateDepth);

		// start moving forward at power set by configuration file.
		State::setProperty(State::desiredPower, gatePower);
		timer.resetTimer();
		pause = pauseTime;
		stage = 1;
		return;
	// check for gate
	case 1:
		if (timer.getCurrentTime() > pauseTime + visionTime)
			stage = 2;
		return;
	case 2:
		// trust that you're gonna go forward for a little
		pause = gateTime;
		stage = 3;
		return;
	case 3:
		isCompleted = true;
		return;
	}
}

bool Gate::check()
{
	return true;
}
