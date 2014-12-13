/*
       _     _                                                  _                   
__   _(_)___(_) ___  _ __      _ __   ___  _ __ _ __ ___   __ _| |  ___ _ __  _ __  
\ \ / / / __| |/ _ \| '_ \    | '_ \ / _ \| '__| '_ ` _ \ / _` | | / __| '_ \| '_ \ 
 \ V /| \__ \ | (_) | | | |   | | | | (_) | |  | | | | | | (_| | || (__| |_) | |_) |
  \_/ |_|___/_|\___/|_| |_|___|_| |_|\___/|_|  |_| |_| |_|\__,_|_(_)___| .__/| .__/ 
                         |_____|                                       |_|   |_|    
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
#include "vision_normal.hpp"

#include <cv.h>
#include <unistd.h>
#include <cerrno>
#include <sys/mman.h>
#include <time.h>
#include <highgui.h>		// for cvSaveImage. GUI doesn't exist

VisionNormal::VisionNormal (libconfig::Config* conf) :
	Vision(conf),
	timpl(new ThreadImpl),
	pool(4, false, 1000)
{
        conf->lookupValue("vision.height", height);
        conf->lookupValue("vision.width", width);
        conf->lookupValue("vision.snapshot_enable", snapshotEnable);
        conf->lookupValue("vision.snapshot_time", snapshotTime);
        conf->lookupValue("vision.camera_id_front", cameraIdFront);
        conf->lookupValue("vision.camera_id_down", cameraIdDown);

	filter = cv::imread("FILTER.jpg");
	cv::cvtColor(filter, filter, CV_BGR2GRAY);
	lprintf("hi: %\n\n", filter.type());
	
	c_front=new Camera(cameraIdFront.c_str(),width,height);
	c_front->pauseCapturing();
	
	c_down=new Camera(cameraIdDown.c_str(),width,height);
	c_down->pauseCapturing();
	
	timpl->m_thread = std::thread(std::bind(&Vision::visionProcess, this));
}

/*
 * Stores the image from the camera requested by cam into i_front or i_down
 */
void VisionNormal::storeImage(HARDWARE cam)
{
	Camera* c = (cam == CAM_FRONT)?c_front:c_down;

	void* buffer = NULL;
	while (buffer == NULL){
		if (c->buffers[c->getIndex()].start != NULL)
		{
			buffer = c->buffers[c->getIndex()].start;
			int i = *(int*)buffer;
		}
	}
	
	
	cv::Mat img(height/2, width/2, CV_8UC3);
	
	// first convert the images from yuv to rgb
	auto yuv2rgb = [](int start, int end, unsigned char* img_data, unsigned char* filt_data, int width, int height, void* buffer)
		{
			int rtmp0, gtmp0, btmp0, rtmp1, gtmp1, btmp1;
			int r0, g0, b0, r1, g1, b1, f0, f1;
			
			for (int i = start; i < end; i++)
			{
				// img_data is the base of the buffer
				// width*height/4 - 1 moves you to the end of the buffer because we need to read backward because the image is upside down
				// i/width gets you to the correct row in the buffer, width/2 is the length of a row in YUYV blocks
				// i%(width/2) gets you to the correct column
				unsigned char* bp = img_data + 3*(width*height/4 - 1 - (i%(width/2) + (i/width)*(width/2)));
			
				// i/(width/2) figures out which row we are on in the raw image
				// if we are on an even row in the raw image, then the row in the rgb image has not been cleared, so we have to
				// set the values to 0
				if ((i / (width/2))%2 == 0)
				{
					bp[2] = 0;
					bp[1] = 0;
					bp[0] = 0;
				}
				
			
				int packed = *((int*)buffer + i);
				// second & is to strip sign
				// 2 pixels share the same u and v values and have different y values
				int y0 = ((char) (packed        & 0xFF)) & 0xFF;
				int y1 = ((char)((packed >> 16) & 0xFF)) & 0xFF;
				int u  = ((char)((packed >> 8)  & 0xFF)) & 0xFF;
				int v  = ((char)((packed >> 24) & 0xFF)) & 0xFF;
								
				y0 = 75 * (y0 - 16);
				y1 = 75 * (y1 - 16);
				u = u - 128;
				v = v - 128;

				// its doing:
				// r = y + 102v, g = y - 25u - 52v, b = y + 129u, but optimized for 2 calculations
				
				rtmp0 = 102*v;
				gtmp0 = -25*u - 52*v;
				btmp0 = 129*u;
				
				rtmp1 = y1 + rtmp0;
				gtmp1 = y1 + gtmp0;
				btmp1 = y1 + btmp0;
				
				rtmp0 += y0;
				gtmp0 += y0;
				btmp0 += y0;

				r0 = (rtmp0 >> 6);
				g0 = (gtmp0 >> 6);
				b0 = (btmp0 >> 6);

				r1 = (rtmp1 >> 6);
				g1 = (gtmp1 >> 6);
				b1 = (btmp1 >> 6);
				
				f0 = filt_data[height*width - 2*i - 1];
				f1 = filt_data[height*width - 2*i - 2];
				
				int ave0 = (2*MAX3(r0,g0,b0)+r0+g0+b0)/5;
				int ave1 = (2*MAX3(r1,g1,b1)+r1+g1+b1)/5;
				
				r0 = ave0 + (r0 - ave0)*f0/128;
				g0 = ave0 + (g0 - ave0)*f0/128;
				b0 = ave0 + (b0 - ave0)*f0/128;
				
				r1 = ave1 + (r1 - ave1)*f1/128;
				g1 = ave1 + (g1 - ave1)*f1/128;
				b1 = ave1 + (b1 - ave1)*f1/128;
				
				r0 = THRESHOLD(0,r0,255);
				g0 = THRESHOLD(0,g0,255);
				b0 = THRESHOLD(0,b0,255);
				r1 = THRESHOLD(0,r1,255);
				g1 = THRESHOLD(0,g1,255);
				b1 = THRESHOLD(0,b1,255);
				
				bp[2] += (r0 + r1)/4;
				bp[1] += (g0 + g1)/4;
				bp[0] += (b0 + b1)/4;
			}
		};
	
	// in YUV422 4 bytes represent 2 pixels, so 1 int is 2 pixels, so we need half as many ints as pixels
	int image_size = height*width/2;
	
	pool.add_task(yuv2rgb,              0,   image_size/4, img.ptr(), filter.ptr(), width, height, buffer);
	pool.add_task(yuv2rgb,   image_size/4,   image_size/2, img.ptr(), filter.ptr(), width, height, buffer);
	pool.add_task(yuv2rgb,   image_size/2, 3*image_size/4, img.ptr(), filter.ptr(), width, height, buffer);
	pool.add_task(yuv2rgb, 3*image_size/4,     image_size, img.ptr(), filter.ptr(), width, height, buffer);

	pool.wait();

	// next figure out how the colors are going to be scaled to enhance the image
	unsigned char r_table[256], b_table[256], g_table[256], colors[6], colorsOut[6];
	scaleColors(img, r_table, g_table, b_table);
	
	((cam == CAM_FRONT)?raw_front:raw_down) = img; 	// saves a copy of the raw image, which is needed if we are going to
							// save the picture later
	
	cv::Mat enhanced(img.size(), CV_8UC3);
	// next enhance all of the colors

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

void VisionNormal::scaleColors(cv::Mat src, unsigned char* red_table, unsigned char* green_table, unsigned char* blue_table)
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

VisionNormal::~VisionNormal()
{
	timpl->stop_requested = true;
	timpl->m_thread.join();
	lprintf("Vision has stopped\n");

	delete c_front;
	delete c_down;
}

void VisionNormal::initializeCamera(HARDWARE cam)
{
	Camera* &target = (cam == CAM_FRONT)?c_front:c_down;
	Camera* &other = (cam == CAM_FRONT)?c_down:c_front;
	
	if (target->isCapturing)
	{
		return;
	}
	
	if (other->isCapturing)
	{
		
		other->pauseCapturing();
	}
	
	target->resumeCapturing();
}

void VisionNormal::visionProcess()
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
			if (!c_front->isCapturing)
				initializeCamera(CAM_FRONT);
			storeImage(CAM_FRONT);
			requestImage &= ~CAM_FRONT;
		}
		if (requestImage & CAM_DOWN)
		{
			if (!c_down->isCapturing)
				initializeCamera(CAM_DOWN);
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
