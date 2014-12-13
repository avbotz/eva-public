/*
 _               _                 
| | ___   __ _  | |__  _ __  _ __  
| |/ _ \ / _` | | '_ \| '_ \| '_ \ 
| | (_) | (_| |_| | | | |_) | |_) |
|_|\___/ \__, (_)_| |_| .__/| .__/ 
         |___/        |_|   |_|    
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
#ifndef LOG_H_
#define LOG_H_

#include <cstdlib>
#include <ctime>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>

namespace Log
{
	extern std::string directory;
	extern std::ofstream ofs;
	extern std::recursive_mutex m_mutex;
};

extern const char *git_version, *build_date, *build_time;

void lprintf(const char* s);

template<typename T, typename... Args> inline void lprintf(const char *s, T value, Args... args)
{
	if (!Log::ofs.is_open())
	{
		std::cout << &Log::ofs << std::endl;
	}
	std::lock_guard<std::recursive_mutex> lock(Log::m_mutex);
	while (*s)
	{
		if (*s == '%')
		{
			if (*(s + 1) == '%')
			{
				++s;
				if (!*s)
				{
					break;
				}
			}
			else
			{
				std::cout << value;
				Log::ofs << value;  // streams will automatically flush when needed
				// call even when *s == 0 to detect extra arguments
				lprintf(s + 1, args...); 
				return;
			}
		}
		Log::ofs << *s;
		std::cout << *s++;
	}
	// silently fail (extra arguments)
}

int lopen();

std::string getLogDirectory();

#endif
