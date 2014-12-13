/*
                                      _                 
  ___ __ _ _ __ ___   ___ _ __ __ _  | |__  _ __  _ __  
 / __/ _` | '_ ` _ \ / _ \ '__/ _` | | '_ \| '_ \| '_ \ 
| (_| (_| | | | | | |  __/ | | (_| |_| | | | |_) | |_) |
 \___\__,_|_| |_| |_|\___|_|  \__,_(_)_| |_| .__/| .__/ 
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
#ifndef CAMERA_H__
#define CAMERA_H__

#include "threadimpl.hpp"
#include "types.hpp"
#include "log.hpp"

struct buffer
{
	void* start;
	size_t length;
};

class Camera
{
public:
	Camera(const char* name, int w, int h);
	~Camera();
	
	void* getImageBuffer();
	int getIndex();
	
	void pauseCapturing();
	void resumeCapturing();

	bool isCapturing;
	
	buffer* buffers;
private:

	void openDevice();
	void initDevice();
	void initMMap();
	void startCapturing();

	void stopCapturing();
	void uninitDevice();
	void closeDevice();

	int readFrame();
	void runVision();

	void errno_exit(const char *s);
	int xioctl(int request, void *arg);

	int fd;
	char devName[20];
	IO_METHOD io;
	
	void* latestBuffer;
	unsigned int numBuffers;
	int dropFrames;
	ThreadImpl* timpl;
	int width, height;
	int index;
};


#endif
