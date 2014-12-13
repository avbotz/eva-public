/*
                         _       _                 
 ___  ___  __ _ _ __ ___| |__   | |__  _ __  _ __  
/ __|/ _ \/ _` | '__/ __| '_ \  | '_ \| '_ \| '_ \ 
\__ \  __/ (_| | | | (__| | | |_| | | | |_) | |_) |
|___/\___|\__,_|_|  \___|_| |_(_)_| |_| .__/| .__/ 
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
/* search is sort of a hybrid task
 * it can take control of the sub and make it move
 * but it also runs a thread to check if any task
 * has been found
 */
#ifndef SEARCH_H__
#define SEARCH_H__

#include "task.hpp"


class Search : public Task
{
public:
	Search(libconfig::Config *conf, Task** tList);
	~Search();

	void action();
    bool check();

	void startSearchService(Vision* v);
	
	// holds the index of the task that the search service finds so that eva can grab it
	int taskFound;

private:

	int panHeading[3];
	void advancePanHeading();
	int currPanHeading;


	int checkPossible();
	
	Task** taskList;

	int timeout;
	int searchPower;
	
	// for timeouts
	Timer timer;
};

#endif
