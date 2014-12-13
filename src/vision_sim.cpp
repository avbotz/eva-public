/*
       _     _                     _                             
__   _(_)___(_) ___  _ __      ___(_)_ __ ___    ___ _ __  _ __  
\ \ / / / __| |/ _ \| '_ \    / __| | '_ ` _ \  / __| '_ \| '_ \ 
 \ V /| \__ \ | (_) | | | |   \__ \ | | | | | || (__| |_) | |_) |
  \_/ |_|___/_|\___/|_| |_|___|___/_|_| |_| |_(_)___| .__/| .__/ 
                         |_____|                    |_|   |_|    
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
#include "vision_sim.hpp"

#include <cv.h>
#include <unistd.h>
#include <cerrno>
#include <sys/mman.h>
#include <time.h>
#include <highgui.h>		// for cvSaveImage. GUI doesn't exist

VisionSim::VisionSim (libconfig::Config* conf) :
	Vision(conf),
	timpl(new ThreadImpl),
	pool(4, false, 1000)
{
    conf->lookupValue("vision.height", height);
    conf->lookupValue("vision.width", width);
    conf->lookupValue("vision.snapshot_enable", snapshotEnable);
    conf->lookupValue("vision.snapshot_time", snapshotTime);
	conf->lookupValue("visionsim.camera_filename", camera_filename);
	
	timpl->m_thread = std::thread(std::bind(&Vision::visionProcess, this));
}
void VisionSim::storeImage(HARDWARE cam)
{
	cv::Mat img;

	while(!((img.rows > 0) && (img.cols > 0)))
	{
		try
		{
			img = cv::imread(camera_filename, CV_LOAD_IMAGE_COLOR);
		}
		catch(std::exception &e) {}
	}

	// crop image to get either the front or down camera
	cv::Rect crop_view;
	if (cam == CAM_FRONT)
	{
		crop_view = cv::Rect(0, 0, img.cols, img.rows/2);
	}
	else
	{
		crop_view = cv::Rect(0, img.rows/2, img.cols, img.rows/2);
	}
	cv::Mat cropped;
	cv::Mat(img, crop_view).copyTo(img);

	unsigned char r_table[256], b_table[256], g_table[256], colors[6], colorsOut[6];
	scaleColors(img, r_table, g_table, b_table);

	((cam == CAM_FRONT)?raw_front:raw_down) = img;

	cv::Mat enhanced(img.size(), CV_8UC3);

	auto enhance_colors = [](int start, int end, unsigned char* ip, unsigned char* op, unsigned char* red_table, unsigned char* green_table, unsigned char* blue_table)
		{
			for (int i = start; i < end; i++)
			{	
				op[3*i  ] = blue_table[ip[3*i]];
				op[3*i+1] = green_table[ip[3*i+1]];
				op[3*i+2] = red_table[ip[3*i+2]];
			}
		};

	int num_pixels = img.cols*img.rows;

	pool.add_task(enhance_colors,              0,   num_pixels/4, img.ptr(), enhanced.ptr(), r_table, g_table, b_table);
	pool.add_task(enhance_colors,   num_pixels/4,   num_pixels/2, img.ptr(), enhanced.ptr(), r_table, g_table, b_table);
	pool.add_task(enhance_colors,   num_pixels/2, 3*num_pixels/4, img.ptr(), enhanced.ptr(), r_table, g_table, b_table);
	pool.add_task(enhance_colors, 3*num_pixels/4,     num_pixels, img.ptr(), enhanced.ptr(), r_table, g_table, b_table);

	pool.wait();

	((cam == CAM_FRONT)?i_front:i_down) = enhanced;
}

void VisionSim::scaleColors(cv::Mat src, unsigned char* red_table, unsigned char* green_table, unsigned char* blue_table)
{
	auto getColorExtremes = [](int width, int height, unsigned char* data, int offset, int interval, unsigned char* table)
		{
			int numSamples = (width/interval)*(height/interval);
			unsigned char sample[numSamples];

			int p005 = numSamples*.005f;
			int p995 = numSamples*.995f;

			int k = 0;
			int l = offset;
			// sampling every nth row and column
			for (int i = 0; i < height; i+=interval)
			{
				for (int j = 0; j < width; j+=interval)
				{
					sample[k] = data[3*(i*width+j)+offset];
					k++;
				}
			}

			int c005 = selectKth(sample, 0, numSamples - 1, p005);
			int c995 = selectKth(sample, 0, numSamples - 1, p995);

			int range = c995 - c005, min = 0, max = 255;
			int out_range;

			// we want to scale the out range so that we dont always go from 0 to 255
			out_range = range * ((255.0f/range - 1)/(1+exp(-(range-20)/2.0f)) + 1) + 0.5f;

			max = (c005+c995)/2 + out_range/2;
			min = (c005+c995)/2 - out_range/2;

			if (max > 255)
			{
				min -= (max - 255);
				max = 255;
			}
			else if (min < 0)
			{
				max += -min;
				min = 0;
			}

			if (range == 0)
			{
				for (int i = 0; i < 256; i++)
				{
					table[i] = i;
				}
			}

			else
			{
				for (int i = 0; i < 256; i++)
				{
					table[i] = THRESHOLD(0, (i - c005) * (max-min) / (c995-c005) + min, 255);
				}
			}

		};

	pool.add_task(getColorExtremes, src.cols, src.rows, src.ptr(), 0, 10,  blue_table);
	pool.add_task(getColorExtremes, src.cols, src.rows, src.ptr(), 1, 10, green_table);
	pool.add_task(getColorExtremes, src.cols, src.rows, src.ptr(), 2, 10,   red_table);

	pool.wait();

	return;
}

VisionSim::~VisionSim()
{
	timpl->stop_requested = true;
	timpl->m_thread.join();
	lprintf("Vision has stopped\n");
}

void VisionSim::visionProcess()
{
	while (!timpl->stop_requested)
	{
		std::unique_lock<std::mutex> lock(timpl->m_mutex);	// we want to unlock this asap because it will block the main thread
		if (imagesToSave.empty())
		{
			lock.unlock();
			Timer::sleep(3);
		}
		else
		{
			// save one image
			std::tuple<std::string, cv::Mat> str_img = imagesToSave.front();
			imagesToSave.pop();
			lock.unlock();
			cv::imwrite(std::get<0>(str_img).c_str(), std::get<1>(str_img));
		}

		if (requestImage & CAM_FRONT)
		{
			storeImage(CAM_FRONT);
			requestImage &= ~CAM_FRONT;
		}
		if (requestImage & CAM_DOWN)
		{
			storeImage(CAM_DOWN);
			requestImage &= ~CAM_DOWN;
		}
	}

	while (!imagesToSave.empty())
	{
		std::tuple<std::string, cv::Mat> str_img = imagesToSave.front();
		cv::imwrite(std::get<0>(str_img).c_str(), std::get<1>(str_img));
		imagesToSave.pop();
	}
}
