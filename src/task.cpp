/*
 _            _
| |_ __ _ ___| | __  ___ _ __  _ __
| __/ _` / __| |/ / / __| '_ \| '_ \
| || (_| \__ \   < | (__| |_) | |_) |
 \__\__,_|___/_|\_(_)___| .__/| .__/
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
#include "task.hpp"

Task::Task(libconfig::Config *c, char request):
	isCompleted(false),
	pause(0),
	hardware_request(request),
	stage(0),
	timer(),
	conf(c),
	isAdvanceConditionMet(false),
	isRegressConditionMet(false),
	forceSaveFrame(false)
{
}

Task::~Task()
{
}

void Task::turn(int degrees)
{
	State::setProperty(State::desiredHeading, State::getEquivalentAngle(State::getProperty(State::currentHeading) + degrees));
}

void Task::labelStage(std::string name, int stage)
{
	if (stage == -1)
	{
		stageName.push_back(name);
	}
	else
	{
		if (stageName.size() >= stage)
		{
			stageName.at(stage - 1) = name;
		}
	}
}

void Task::setAdvRegTime(int advTime, int regTime)
{
	advanceTime = advTime;
	regressTime = regTime;
}

int Task::nextStage(bool advanceCondition, bool regressCondition, int forceStage)
{
	if (forceStage != -1)
	{
		stage = forceStage;
		isAdvanceConditionMet = false;
		isRegressConditionMet = false;
	}
	else
	{
		if (advanceCondition)
		{
			if (isAdvanceConditionMet)
			{
				if (advanceTimer.getCurrentTime() > advanceTime)
				{
					// condition has been met for a while, go to next stage
					stage++;
					isAdvanceConditionMet = false;
					isRegressConditionMet = false;
				}
			}
			else
			{
				// just met the advance condition
				isAdvanceConditionMet = true;
				advanceTimer.resetTimer();
			}
		}
		else
		{
			isAdvanceConditionMet = false;
		}

		if (regressCondition)
		{
			if (isRegressConditionMet)
			{
				if (advanceTimer.getCurrentTime() > regressTime)
				{
					// condition has been met for a while, go back
					stage--;
					isAdvanceConditionMet = false;
					isRegressConditionMet = false;
				}
			}
			else
			{
				// just met regress condition
				isRegressConditionMet = true;
				advanceTimer.resetTimer();
			}
		}
		else
		{
			isRegressConditionMet = false;
		}
	}
	return stage;
}
