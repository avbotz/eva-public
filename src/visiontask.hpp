/*
       _     _             _            _      _                 
__   _(_)___(_) ___  _ __ | |_ __ _ ___| | __ | |__  _ __  _ __  
\ \ / / / __| |/ _ \| '_ \| __/ _` / __| |/ / | '_ \| '_ \| '_ \ 
 \ V /| \__ \ | (_) | | | | || (_| \__ \   < _| | | | |_) | |_) |
  \_/ |_|___/_|\___/|_| |_|\__\__,_|___/_|\_(_)_| |_| .__/| .__/ 
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
#ifndef VISION_TASK_H__
#define VISION_TASK_H__

#include <list>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include "task.hpp"
#include "blob_detection.hpp"
#include "threadpool/pool.hpp"

class VisionTask : public Task
{
public:
	VisionTask(libconfig::Config *conf, HARDWARE request);
	virtual ~VisionTask();

	virtual void action() = 0;
	virtual bool check() = 0;

	// src should basically be grayscale image with type CV_8UC1 (8 bits unsigned 1 channel)
	// it doesnt IPL image for speed since blob detection will be done on grayscale images
	// sigma is the base value used for the gaussian blur
	// k is the step; first blur will be sigma, then k*sigma, then k^2*sigma ...
	// iterations is the number of differences that will be created
	// are we looking for the max or the min, since both are extrema
	std::vector<Blob*> blobDetection(cv::Mat src, int color, int channels);
	
	// this is public and exists so that EVA can take care of saving frames since it has to be done in every vision task
	cv::Mat processedImage;
	
	// takes a 3 channel RGB image and returns a 1 channel image with the hue value of all of the pixels
	cv::Mat generateHueMap(cv::Mat rgb);
	
	bool snapshotEnable;
	int snapshotTime;
protected:
	int height, width, masterDownsampleRatio, pic_height, pic_width;
	int fov_horizontal, fov_vertical;
	
	// this function is here because multiple visiontasks use it and theyre doing the same panning motion
	void initializePan(int center, int delta);
	int panHeading[3];
	int currentPanHeading;
	int numberOfPanHeadings;
	void goToNextPanHeading();
	
	// the amount of time it will take to stop the sub when going at 70/130 power
	int stopTime();
	int stopTimeScale;
	
	cv::Mat downsample(cv::Mat src, int ratio);
	// sample every n^2th pixel
	
	// figures out how far an object is based on its width in pixels vs true height in inches
	float objectDistance (int px, int in, bool horizontal);
	
	// memory management
	void freeBlobs(std::vector<Blob*> &list);
	void freeBlobs(Blob** list, int length);
	
	std::vector<Blob*> consolidateBlobLists(std::vector<Blob*> a, std::vector<Blob*> b, int channels);
	
	int getPointHeading(cv::Point p);

	threadpool::pool pool;
};

#endif
