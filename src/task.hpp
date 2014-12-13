/*
 _            _      _
| |_ __ _ ___| | __ | |__  _ __  _ __
| __/ _` / __| |/ / | '_ \| '_ \| '_ \
| || (_| \__ \   < _| | | | |_) | |_) |
 \__\__,_|___/_|\_(_)_| |_| .__/| .__/
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
#ifndef TASK_H__
#define TASK_H__

#include <libconfig.h++>
#include <string.h>
#include <condition_variable>

#include "state.hpp"
#include "types.hpp"
#include "timer.hpp"
#include "log.hpp"

class Task
{
public:
	Task(libconfig::Config *c, char request);
	virtual ~Task();

	// control the sub leading up to and executing the task
	virtual void action() = 0;

	// check if based on the state if it is possible to do this task
	virtual bool check() = 0;

	// set when the task is completed so that eva knows when to go to the next task
	bool isCompleted;

	// does the task want to sleep before action is called again (in increments of 5 ms)
	int pause;

	// what hardware does the task require to function
	// it is of type char to support multiple hardware requests for a single task
	char hardware_request;

	// each task probably is complex and has multiple stages (for example buoy stage 0 is find 1st buoy, stage 1 is ram, etc...)
	int stage;

	// needed by a lot of tasks
	Timer timer;

	libconfig::Config *conf;

	void setAdvRegTime(int advTime, int regTime);
	int nextStage(bool advanceCondition, bool regressCondition, int forceStage = -1);

	int forceSaveFrame;

	std::vector<std::string> stageName;
	void labelStage(std::string name, int stage = -1);
	void turn(int degrees);
protected:
	bool isAdvanceConditionMet, isRegressConditionMet;
	int advanceTime, regressTime;
	Timer advanceTimer;
};

#endif
