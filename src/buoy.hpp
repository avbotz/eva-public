/*
 _                         _                 
| |__  _   _  ___  _   _  | |__  _ __  _ __  
| '_ \| | | |/ _ \| | | | | '_ \| '_ \| '_ \ 
| |_) | |_| | (_) | |_| |_| | | | |_) | |_) |
|_.__/ \__,_|\___/ \__, (_)_| |_| .__/| .__/ 
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
#ifndef BUOY_H__
#define BUOY_H__

#include "visiontask.hpp"

#include <algorithm>

class Buoy : public VisionTask
{
public:
	Buoy(libconfig::Config *conf);
	~Buoy();
    
    void action();
    bool check();

private:
	bool ram();

	void processImage();
	
	enum
	{
		Init,
		Ram,
		Backup,
		ResetHeadingAndDepth,
		MoveOn,
		Finish
	};

	enum BUOY_TYPE
	{
		B_RED = 0,
		B_GREEN,
		B_CHANGING
	};

	unsigned int blobMinArea, blobMaxArea;
	
	int currentBuoy;
	
	int buoyDepth;
	
	Blob* buoy_blob[3];
	unsigned int num_blobs;
	
	int initialHeading;

	int backupTime;
	int moveOnTime;
	int ramArea;
	int ramTime;
	
	int moveOnDepth;
	int headingConstant;
	int depthConstant;

	float hThreshold;
	float vThreshold;

	int endAngleOffset;

	float wallBlobHeight;
	float wallBlobDistanceX;
	float minBuoyCircularity;
	float minBuoyDensity;
	float glareBlobWidth;
	float glareBlobHeight;

	int rb, rg;
	int gr, gb;

	float blobPropRatio;
	float blobPropWidth;
	float blobPropHeight;

	bool haveWeHit[3];

	Timer timeout;
	int rMinPixelIntensity;
	int maxDepth;
	int wMinPixelIntensity;
};

#endif
