/*
 _                           _                 
| |_ _   _ _ __   ___  ___  | |__  _ __  _ __  
| __| | | | '_ \ / _ \/ __| | '_ \| '_ \| '_ \ 
| |_| |_| | |_) |  __/\__ \_| | | | |_) | |_) |
 \__|\__, | .__/ \___||___(_)_| |_| .__/| .__/ 
     |___/|_|                     |_|   |_|    
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
#ifndef TYPES_H__
#define TYPES_H__

// the number of tasks we plan to attempt
#define NUM_TASKS 8	

#include <string.h>

/*
 * Used by Camera (V4L2).
 */
enum IO_METHOD
{
	IO_READ,
	IO_MMAP,
	IO_USERPTR
};

/*
 * Flags to designate different hardware
 * connected to the beagleboard that isnt
 * so that Tasks can request only the
 * hardware they need
 */
enum HARDWARE
{
	NONE = 0x00,
	CAM_FRONT = 0x01,
	CAM_DOWN = 0x02
};

/*
 * An enumeration of the tasks that EVA can perform, listed with nonvision tasks first.
 * This is so that the current task can be identified.
 */

enum TASK_ID
{
	T_NUL = -1,	// used in search to say that nothing has been found; the current task cannot be set to T_NUL
	T_GATE,
	T_SEARCH,
	T_HYDROPHONE,
	T_BUOY,
	T_DROPPER,
	T_LGATE,
	T_PATH,
	T_TORPEDO,
	T_PARTY
};

/*
 * An array of strings, each corresponds to a tasks's name in the config file
 * It should be in the same order as the task list above
 */

const std::string TASK_NAME[NUM_TASKS] =
{
	"gate",
	"search",
	"hydrophone",
	"buoy",
	"dropper",
	"l_gate",
	"path",
	"torpedo",
};

/*
 * The sub has three kill states: alive, dead, and stopped (via SIGINT)
 */
enum RUN_STATE
{
	R_ALIVE = 0x00,
	R_KILLED = 0x01,
	R_STOPPED = 0x02
};

/*
 * The dropper can either have dropped none, one or two (all) of the canisters
 */
enum DROP_STATE
{
	D_ZERO = 0,
	D_ONE = 1,
	D_TWO = 2
};

#define IM_HERE lprintf("EVA is at %:%\n",__FILE__,__LINE__);

// thresholds a number to between 0 and 255
#define THRESHOLD(a,x,b) ((x < a) ? a : ((x > b) ? b : x))

#ifndef MAX
	#define MAX(a,b) ((a>b)?(a):(b))
#endif

#ifndef MAX3
	#define MAX3(a,b,c) ((MAX(a,b)>c)?(MAX(a,b)):(c))
#endif

#ifndef MIN
	#define MIN(a,b) ((a<b)?(a):(b))
#endif

#ifndef MIN3
	#define MIN3(a,b,c) ((MIN(a,b)<c)?(MIN(a,b)):(c))
#endif

#ifndef MID3
	#define MID3(a,b,c) ((a<b) ? ((c>b) ? (MAX(a,c)) : (b)) : ((a>c) ? MAX(b,c) : (a)))
#endif

// The following macros assume no 2 arguments are equal

#ifndef MAX_WHICH
	#define MAX_WHICH(a,b) ((a>b)?(0):(1))
#endif

#ifndef MAX3_WHICH
	#define MAX3_WHICH(a,b,c) ((MAX(a,b)>c)?(MAX_WHICH(a,b)):(2))
#endif

template <typename T>
T selectKth(T* arr, int start, int finish, int k)
{
	// quickselect
	
	// the only way for these 2 numbers to converge
	// is if they converge on k due to the recursive
	// call at the end
	if (start == finish)
	{
		return arr[k];
	}
	
	// use last value in list as pivot
	int pivot = arr[finish];
	// storeIndex holds the place where there is a number
	// greater than or equal to pivot which wants to be switched
	// into the back of the list but needs to wait for a small
	// number in the back of the list
	int storeIndex = start;
	
	for (int i = start; i < finish; i++)
	{
		if (arr[i] <
			pivot)
		{
			int temp = arr[i];
			arr[i] = arr[storeIndex];
			arr[storeIndex] = temp;
			storeIndex++;
		}
	}
	
	// put the pivot in the final place
	arr[finish] = arr[storeIndex];
	arr[storeIndex] = pivot;

	// if our pivot is in the right place in the list
	// so if its in the kth spot then it is actually the 
	// kth value
	if (storeIndex == k)
	{
		return arr[k];
	}
	// if our pivot is above the kth value, the kth value
	// is in the bottom half of the list
	if (k < storeIndex)
	{
		return selectKth(arr, start, storeIndex - 1, k);
	}
	// if our pivot is below the kth value, the kth value
	// is in the top half of the list
	else
	{
		return selectKth(arr, storeIndex + 1, finish, k);
	}
}

#endif
