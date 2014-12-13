/*
                   _                 
  _____   ____ _  | |__  _ __  _ __  
 / _ \ \ / / _` | | '_ \| '_ \| '_ \ 
|  __/\ V / (_| |_| | | | |_) | |_) |
 \___| \_/ \__,_(_)_| |_| .__/| .__/ 
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

#ifndef EVA_H__
#define EVA_H__

#include <libconfig.h++>
#include "state.hpp"
#include "serial.hpp"
#include "serial_mbed.hpp"
#include "task.hpp"
#include "search.hpp"
#include "gate.hpp"
#include "hydrophone.hpp"
#include "buoy.hpp"
#include "dropper.hpp"
#include "l_gate.hpp"
#include "path.hpp"
#include "torpedo.hpp"
#include "types.hpp"
#include "timer.hpp"
#include "log.hpp"
#include "vision_normal.hpp"
#include "vision_sim.hpp"


class eva
{
public:
	eva(libconfig::Config *config);
	~eva();
	
	friend void wall_e(int);

	void runTasks();
	int waitForWall_e();

	static volatile int stopEVA;

private:
	void printDisclaimer();
	
	void refreshTaskList();

	// waits for a hardware request to complete before calling someones action function
	void waitForHardwareRequest(char hardware);

	libconfig::Config *conf;
	State *evaState;
	
	SerialMbed *mbed;

	// TODO figure out if we actually need an array to hold all of the tasks at once?
	// probably not but it definitely is a helluva lot more convinient
	Task* taskList[NUM_TASKS];
	Task* instantiator(TASK_ID t);
	TASK_ID identify(std::string name);

	Vision* vision;

	// whether the search service should run. Main sets it to true to start search
	// service sets itself to false if something is found
	bool searchService;
	
	// used to calculate FPS
	long long lastFrameSaveTime;
	
	// used to tell when a task changes stages
	int lastStage;
};

eva* barracuda;

#endif
