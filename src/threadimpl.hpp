/*
 _   _                        _ _                 _   _                 
| |_| |__  _ __ ___  __ _  __| (_)_ __ ___  _ __ | | | |__  _ __  _ __  
| __| '_ \| '__/ _ \/ _` |/ _` | | '_ ` _ \| '_ \| | | '_ \| '_ \| '_ \ 
| |_| | | | | |  __/ (_| | (_| | | | | | | | |_) | |_| | | | |_) | |_) |
 \__|_| |_|_|  \___|\__,_|\__,_|_|_| |_| |_| .__/|_(_)_| |_| .__/| .__/ 
                                           |_|             |_|   |_|    
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
#ifndef THREAD_IMPL_H__
#define THREAD_IMPL_H__

#include <thread>
#include <mutex>

/*
 * Our special boost implementation.
 * We include a mutex for when we want to lock objects.
 * We also include a boost::thread object for things
 * that require threading, like Vision, Camera, and Serial.
 * Finally, stop_requested lets the object containing
 * the BoostImpl know that it should terminate ASAP.
 */
struct ThreadImpl
{
	ThreadImpl() :
		stop_requested(false)
	{

	}

	std::mutex m_mutex;
	std::thread m_thread;
	volatile bool stop_requested;
};

#endif
