/*
       _     _             _            _                      
__   _(_)___(_) ___  _ __ | |_ __ _ ___| | __  ___ _ __  _ __  
\ \ / / / __| |/ _ \| '_ \| __/ _` / __| |/ / / __| '_ \| '_ \ 
 \ V /| \__ \ | (_) | | | | || (_| \__ \   < | (__| |_) | |_) |
  \_/ |_|___/_|\___/|_| |_|\__\__,_|___/_|\_(_)___| .__/| .__/ 
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
#include "visiontask.hpp"
#include <exception>
#include <stdint.h>

VisionTask::VisionTask(libconfig::Config *c, HARDWARE request) : Task(c, request),
	numberOfPanHeadings(3),
	pool(4, false, 1000)
{
    conf->lookupValue("vision.height", height);
    conf->lookupValue("vision.width", width);
	conf->lookupValue("vision.fov_horizontal", fov_horizontal);
	conf->lookupValue("vision.fov_vertical", fov_vertical);
    conf->lookupValue("vision.snapshot_enable", snapshotEnable);
    conf->lookupValue("vision.snapshot_time", snapshotTime);
	conf->lookupValue("vision.master_downsample_ratio", masterDownsampleRatio);
	conf->lookupValue("visiontask.stop_time_scale", stopTimeScale);
	
	pic_height = height/masterDownsampleRatio;
	pic_width = width/masterDownsampleRatio;
	
	processedImage.create(pic_height, pic_width, CV_8UC3);
}

VisionTask::~VisionTask()
{
}

std::vector<Blob*> VisionTask::blobDetection(cv::Mat src, int color, int channels)
{
	std::vector<Blob*> blobList = blob_detection(src, color, channels);
	return blobList;
}


cv::Mat VisionTask::generateHueMap(cv::Mat rgb)
{
	cv::Mat hue(height, width, CV_16SC1);
	
	for (int i = 0; i < height*width; i++)
	{
		unsigned char* rgb_data = (unsigned char*)rgb.data + i*3;
		int16_t* hue_data = (int16_t*)hue.data + i*3;
		unsigned char r = rgb_data[2];
		unsigned char g = rgb_data[1];
		unsigned char b = rgb_data[0];
		
		if (r == g && g == b)
		{
			*hue_data = -1;
		}
		if (r >= b)
		{
			if (r >= g)
			{
				if (g >= b)
				{
					// red-yellow
					*hue_data = 60*(g-b)/(r-b);
				}
				else
				{
					// magenta-red
					*hue_data = 360 - 60*(b-g)/(r-g);
				}
			}
			else
			{
				// yellow-green
				*hue_data = 120 - 60*(r-b)/(g-b);
			}
		}
		else
		{
			if (b >= g)
			{
				if (g >= r)
				{
					// cyan-blue
					*hue_data = 240 - 60*(g-r)/(b-r);
				}
				else
				{
					// blue-magenta
					*hue_data = 240 + 60*(r-g)/(b-g);
				}
			}
			else
			{
				// green-cyan
				*hue_data = 120 + 60*(b-r)/(g-r);
			}
		}
	}
	return hue;
}

cv::Mat VisionTask::downsample(cv::Mat src, int ratio)
{
	cv::Mat dst(src.rows/ratio, src.cols/ratio, CV_8UC3);
	if (ratio == 1)
	{
		dst = cv::Mat(src);
	}
	else
	{
		for (int c = 0; c < 3; c++)
		{
			for (int i = 0; i < dst.rows; i++)
			{
				for (int j = 0; j < dst.cols; j++)
				{
					int sum = 0;
					for (int k = 0; k < ratio; k++)
					{
						for (int l = 0; l < ratio; l++)
						{
							sum += src.at<unsigned char>(i*ratio+k, j*ratio+l, c);
						}
					}
					// find the average
					sum /= (ratio*ratio);
					dst.at<unsigned char>(i,j,c) = (unsigned char)sum;
				}
			}
		}
	}
	return dst;
}

std::vector<Blob*> VisionTask::consolidateBlobLists(std::vector<Blob*> a, std::vector<Blob*> b, int channels)
{
	std::list<Blob*> result;
	for (Blob* const &blob: a)
	{
		if (blob != nullptr)
			result.insert(result.end(), new Blob(*blob));
	}
	for (Blob* const &blob: b)
	{
		if (blob != nullptr)
			result.insert(result.end(), new Blob(*blob));
	}
	int num_combos;
	do
	{
		// keep on making passes while combinations keep happening
		num_combos = 0;
		for (std::list<Blob*>::iterator it_i = result.begin(); it_i != result.end(); it_i++)
		{
			for (std::list<Blob*>::iterator it_j = result.begin(); it_j != result.end(); it_j++)
			{
				if (it_i == it_j)
					continue;
				
				if ((*it_i)->overlapsWith(*it_j))
				{
					// add the new blob to the end of the list, and remove
					// the two that were combined to make it
					result.insert(result.begin(), new Blob(*it_i, *it_j, channels));
					delete *it_i;
					delete *it_j;
					it_i = result.erase(it_i);
					it_j = result.erase(it_j);
					it_i--;
					it_j--;
					num_combos++;
				}
			}
		}
	}
	while (num_combos != 0);
	
	std::vector<Blob*> vec_result;
	vec_result.insert(vec_result.end(), result.begin(), result.end());
	// sorts in order of decreasing area
	std::sort(vec_result.begin(), vec_result.end(), 
		[](Blob* const a, Blob* const b)->bool
		{
			return (a->area > b->area);
		}
	);
	return vec_result;
}

void VisionTask::goToNextPanHeading()
{
	currentPanHeading++;
	switch (currentPanHeading%4)
	{
	case 0:
	case 2:
		State::setProperty(State::desiredHeading, panHeading[0]); break;
	case 1:
		State::setProperty(State::desiredHeading, panHeading[1]); break;
	case 3:
		State::setProperty(State::desiredHeading, panHeading[2]); break;
	}
}

void VisionTask::freeBlobs(std::vector<Blob*> &list)
{
	list.erase(list.begin(), list.end());
}

void VisionTask::freeBlobs(Blob** list, int length)
{
	for (int i = 0; i < length; i++)
	{
		delete list[i];
		list[i] = nullptr;
	}
}

float VisionTask::objectDistance (int px, int in, bool horizontal)
{
	// figure out what is half of the angle to object fills
	float angle = (float)(horizontal?fov_horizontal:fov_vertical) * px/2 /
											 (horizontal?pic_width:pic_height);
	// use trig
	return ((float)in)/2 / tan(CV_PI / 180 * angle);
}

void VisionTask::initializePan(int center, int delta)
{
	panHeading[0] = center;
	panHeading[1] = State::getEquivalentAngle(center - delta);
	panHeading[2] = State::getEquivalentAngle(center + delta);
}

int VisionTask::stopTime()
{
	return abs(State::getProperty(State::currentPower) - 100)*stopTimeScale;
}

int VisionTask::getPointHeading(cv::Point p)
{
	int heading = State::getProperty(State::currentHeading);
	heading += p.x*fov_horizontal/pic_width - fov_horizontal/2;
	return State::getEquivalentAngle(heading);
}
