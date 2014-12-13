/*
             _   _       _                 
 _ __   __ _| |_| |__   | |__  _ __  _ __  
| '_ \ / _` | __| '_ \  | '_ \| '_ \| '_ \ 
| |_) | (_| | |_| | | |_| | | | |_) | |_) |
| .__/ \__,_|\__|_| |_(_)_| |_| .__/| .__/ 
|_|                           |_|   |_|    
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
#ifndef PATH_H__
#define PATH_H__

#include <iostream>
#include <vector>
#include <math.h>

#include <cv.h>
#include <highgui.h>

#include <array>

#include "visiontask.hpp"

class _line
{
public:
    cv::Point pt1, pt2;
    double slope;
    _line(cv::Point firstpoint, cv::Point secondpoint, double riserun) : pt1(firstpoint), pt2(secondpoint), slope(riserun) { }
};


class Path : public VisionTask
{
public:
	Path(libconfig::Config *conf);
	~Path();
	
	void action();
    bool check();

private:
	bool processImage();
	
	void search();
	void center();
	void follow();

	void thresholdColor(int start, int end, unsigned char* ip, unsigned char* processed);
	void detectLines(cv::Mat mat,
									 std::vector<std::array<float,3>>* adjusted_lines,
									 unsigned int* color_quantity);
	
	void toGrayscale();

	int minTotalIntensity;
	int followTime, proceedTime;
	
	int num_paths, preferred_path;
	bool midOfPoolIsCCW;

	int initialHeading;

	bool canSeePath;
	bool startedSearchLeg;

	bool isCheck;

	const cv::Mat *img;
	cv::Mat grayImg;
	
	// the center of gravity of the color
	double cX, cY;
	static int frame;
	
	Timer checkTimer;
	int timeout;
};

#endif
