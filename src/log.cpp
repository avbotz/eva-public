/*
 _                               
| | ___   __ _   ___ _ __  _ __  
| |/ _ \ / _` | / __| '_ \| '_ \ 
| | (_) | (_| || (__| |_) | |_) |
|_|\___/ \__, (_)___| .__/| .__/ 
         |___/      |_|   |_|    
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
#include "log.hpp"

namespace Log
{
	std::string directory;
	std::ofstream ofs;
	std::recursive_mutex m_mutex;
};

void lprintf(const char* s)
{
	std::lock_guard<std::recursive_mutex> lock(Log::m_mutex);
	while (*s)
	{
		if (*s == '%')
		{
			++s;  // assume %%, otherwise there aren't enough arguments so we silently fail
			if (!*s)
			{
				break;
			}
		}
		std::cout << *s;
		Log::ofs << *s++;
	}
}

int lopen()
{
	// make the directory for this specific run
	char time_string[64], make_directory[64] = "mkdir -p ", log_name[64];
	time_t current_time = time(nullptr);
	struct tm* ptm = gmtime(&current_time);
	sprintf(time_string, "%02d_%02d_%02d_%02d_%02d_%02d",
	ptm->tm_year - 100, ptm->tm_mon + 1, ptm->tm_mday,
	ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

	Log::directory += "logs/";
	Log::directory += time_string;
	Log::directory += "/";

	strcat(make_directory, Log::directory.c_str());
	system(make_directory);

	// create the log file
	strcpy(log_name, Log::directory.c_str());
	strcat(log_name, "LOG");

	Log::ofs.open(log_name, std::ios::out);

	if (!Log::ofs.is_open())
	{
		// could not open the log file
		std::cout <<"**WARNING** Could not open log file! Won't log any mission data!\n";
	}

	lprintf("\n\nWelcome to EVA.\n");
	lprintf("Version % / Built % %\n\n", git_version, build_date, build_time);
	lprintf("Run began @ %\n", ctime(&current_time));

	return 0;
}

std::string getLogDirectory()
{
	return Log::directory;
}
