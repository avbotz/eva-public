/*
       _     _                                                  _   _                 
__   _(_)___(_) ___  _ __      _ __   ___  _ __ _ __ ___   __ _| | | |__  _ __  _ __  
\ \ / / / __| |/ _ \| '_ \    | '_ \ / _ \| '__| '_ ` _ \ / _` | | | '_ \| '_ \| '_ \ 
 \ V /| \__ \ | (_) | | | |   | | | | (_) | |  | | | | | | (_| | |_| | | | |_) | |_) |
  \_/ |_|___/_|\___/|_| |_|___|_| |_|\___/|_|  |_| |_| |_|\__,_|_(_)_| |_| .__/| .__/ 
                         |_____|                                         |_|   |_|    
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
#ifndef VISION_NORMAL_H__
#define VISION_NORMAL_H__

#include <libconfig.h++>

#include "threadimpl.hpp"
#include "types.hpp"
#include "camera.hpp"
#include "timer.hpp"
#include "log.hpp"
#include "threadpool/pool.hpp"
#include "vision.hpp"


#include <opencv/cxcore.h>
#include <queue>
#include <tuple>


class VisionNormal : public Vision
{
public:
	VisionNormal(libconfig::Config* conf);
	~VisionNormal();

	virtual void visionProcess();

private:
	virtual void storeImage(HARDWARE cam);
	void initializeCamera(HARDWARE cam);
	const char *front_name, *down_name;
	std::string cameraIdFront, cameraIdDown;
	
	// result is an array of size six in it go the results
	// in order b005, g005, r005, b995, g995, r995
	// where 005 refers to the .5 percentile and 995 to the 99.5 percentile
	void scaleColors (cv::Mat src, unsigned char* red_table, unsigned char* green_table, unsigned char* blue_table);
	
	Camera *c_front, *c_down;
	int height, width;

	ThreadImpl* timpl;
	threadpool::pool pool;

	cv::Mat filter;
};

#endif
