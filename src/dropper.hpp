/*
     _                                 _                 
  __| |_ __ ___  _ __  _ __   ___ _ __| |__  _ __  _ __  
 / _` | '__/ _ \| '_ \| '_ \ / _ \ '__| '_ \| '_ \| '_ \ 
| (_| | | | (_) | |_) | |_) |  __/ | _| | | | |_) | |_) |
 \__,_|_|  \___/| .__/| .__/ \___|_|(_)_| |_| .__/| .__/ 
                |_|   |_|                   |_|   |_|    
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
#ifndef DROPPER_H__
#define DROPPER_H__

#include "visiontask.hpp"

class Dropper : public VisionTask
{
public:
	Dropper(libconfig::Config *conf);
	~Dropper();
	
	void action();
	bool check();

private:
	enum
	{
		Init,
		GoToBins,
		FindNextBin,
		ApproachBin,
		CenterOverBin,
		DetectAndDrop,
		Turn,
		Proceed,
		Finish,
	};
	enum class DetectedImage
	{
		Inaccurate,
		Drop,
		DontDrop
	};
	bool goToBins();
	bool alignBins();
	bool findNextBin();
	bool centerOverBin();
	DetectedImage detectImage();

	int findLongAngle(const std::vector<cv::Vec2f>& lines);

	std::vector<Blob*> getBlobs();
	void thresholdColor(int start, int end, unsigned char* imgIn, unsigned char* imgOut, int thresholdValue, int maxVal, int type);

	int minThresBorder;
	int maxThresBin;

	int primaryDepth;

	int minBinArea;

	float density;

	int initialHeading;

	int currentBin;
	bool isValid[4];
	int binPosition[4];

	std::string imgPath;
	std::string imgFile[4];

	int pic_diag;

	float distAllBins;
	float distBin;

	float accuracyThreshold;
	int canthres;
	double canratio;
	int linethres;
	int aligned;

	int goToBinPower;
	int goToBinTime;
	int goToBinAngle;

	int adjustHeadingTime;

	float cameraOffset;

	int endAngleOffset;
	int endPauseTime;

	bool useImageDetection;

	int searchPower;
	int approachPower;
	float slowApproachMultiplier;
	int image_size;

	int minBorderArea;
	int maxBorderArea;
	int minBinBorderDist;
	int maxBinBorderDist;
	int maxBinArea;
};

#endif
