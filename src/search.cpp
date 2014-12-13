/*
                         _                       
 ___  ___  __ _ _ __ ___| |__    ___ _ __  _ __  
/ __|/ _ \/ _` | '__/ __| '_ \  / __| '_ \| '_ \ 
\__ \  __/ (_| | | | (__| | | || (__| |_) | |_) |
|___/\___|\__,_|_|  \___|_| |_(_)___| .__/| .__/ 
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
#include "search.hpp"

Search::Search(libconfig::Config* c, Task** tList) : Task(c, NONE),
	taskFound(T_NUL),
	taskList(tList)
{
	conf->lookupValue("search.timeout", timeout);
	conf->lookupValue("search.power", searchPower);
}

Search::~Search()
{

}

void Search::action() 
{
	switch (stage)
	{
	case 0:
		{
			// clear out the last task found
			taskFound = T_NUL;
			
			// search service causes the sub to proceed slowly and is done when the search service finds something
			State::setProperty(State::desiredPower, searchPower);
			
			panHeading[0] = State::getProperty(State::currentHeading);
			panHeading[1] = State::getEquivalentAngle(panHeading[0] + 20);
			panHeading[2] = State::getEquivalentAngle(panHeading[0] - 20);
			
			currPanHeading = 0;
			State::setProperty(State::desiredHeading, panHeading[currPanHeading]);
			
			lprintf("Search task has started @ %\n", State::getTimeStamp());

			stage = 1;
			
			// sets the hardware request for this search as the combined hardware requests of all things being searched for
			hardware_request = NONE;
			for (int i = 0; i < NUM_TASKS; i++)
			{
				if (taskList[i] != nullptr && i != T_SEARCH)
				{
					hardware_request |= taskList[i]->hardware_request;
					lprintf("Checking TaskID %\n", i);
				}
			}
			
			timer.resetTimer();
			return;
		}
	case 1:
		{
			if (timer.getCurrentTime() > 6000)
			{
				advancePanHeading();
				timer.resetTimer();
			}
			
			taskFound = checkPossible();
			if (taskFound != T_NUL)
			{
				// clean up so its ready for the next time search is called
				stage = 0;
				isCompleted = true;
				return;
			}
		}
	}
}

bool Search::check()
{
	return true;
}

int Search::checkPossible()
{
	for (int i = 0; i < NUM_TASKS; i++)
	{
	// dont look for the search task and
	// only search for instantiated tasks
		if (i != T_SEARCH && taskList[i] != nullptr)
		{
			if (taskList[i]->check())
			{
				lprintf("TaskID % is available.\n", i);
				return i;
				break;
			}
		}
	}
	return T_NUL;
}

void Search::advancePanHeading()
{
	currPanHeading++;
	switch (currPanHeading % 4)
	{
	case 0:
	case 2:
		State::setProperty(State::desiredHeading, panHeading[0]);
		break;
	case 1:
		State::setProperty(State::desiredHeading, panHeading[1]);
		break;
	case 3:
		State::setProperty(State::desiredHeading, panHeading[2]);
		break;
	}
}
