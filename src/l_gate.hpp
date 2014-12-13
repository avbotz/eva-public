/*
 _                 _         _                 
| |     __ _  __ _| |_ ___  | |__  _ __  _ __  
| |    / _` |/ _` | __/ _ \ | '_ \| '_ \| '_ \ 
| |   | (_| | (_| | ||  __/_| | | | |_) | |_) |
|_|____\__, |\__,_|\__\___(_)_| |_| .__/| .__/ 
 |_____|___/                      |_|   |_|    
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
#ifndef L_GATE_H__
#define L_GATE_H__

#include "visiontask.hpp"

class L_gate : public VisionTask
{
public:
	L_gate(libconfig::Config *conf);
	~L_gate();
	
	void action();
    bool check();

private:
	enum
	{
		Init,
		Search,
		Approach,
		Adjust,
		CircleCheck,
		CircleTurn,
		CircleMove,
		Done
	};

	void search();
	void approach();
	void adjust();
	bool processImage(bool check = false);
	void circleCheck();
	void checkAdvance();

	int gateDepth;
	int initPower;
	int approachPower;
	int approachWaitTime;
	int adjustWaitTime;
	int turnHeading;
	int rbPixelIntensity;
	int rgPixelIntensity;
	int rMinBlobArea;
	int grPixelIntensity;
	int gbPixelIntensity;
	int gMinBlobArea;
	int wMinPixelIntensity;

	int advanceTime;

	cv::Mat img;
	cv::Point blobCenter;

	bool checkGreen;

	bool attemptCircle;
	int circlePower1;
	int circlePower2;
	int circleIncrement;
	int circleTurnTime;
	int circleMoveTime;

	int totalTurnAngle;

	int initialAngle;
	int endPower;
	int endPause;
	int endAngleOffset;

	float boxRatio;

	cv::Point redCentroid;

	bool deadreck;
};

#endif
