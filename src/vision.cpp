/*
       _     _                               
__   _(_)___(_) ___  _ __    ___ _ __  _ __  
\ \ / / / __| |/ _ \| '_ \  / __| '_ \| '_ \ 
 \ V /| \__ \ | (_) | | | || (__| |_) | |_) |
  \_/ |_|___/_|\___/|_| |_(_)___| .__/| .__/ 
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
#include "vision.hpp"

#include <unistd.h>
#include <cerrno>
#include <sys/mman.h>
#include <time.h>

Vision::Vision (libconfig::Config* conf) :
	requestImage(0),
	skippedFrames(0),
	snapshotEnable(false),
	timpl(new ThreadImpl),
	picIndex(0)
{
}

void Vision::yuv2rgb(int start, int end, unsigned char* img_data,
	unsigned char* filt_data, int width, int height, void* buffer)
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

		// it's doing:
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
}

Vision::~Vision()
{

}

/*/
 * Converts image to RGB, then saves it as a JPG. Passes in the processed image. 
 * Save the enhanced and normal images with it
 */
void Vision::saveFrame(int cam, const cv::Mat img)
{
	char fileNameR[64];
	char fileNameE[64];
	char fileNameP[64];

	sprintf(fileNameR, "%d.jpg" , picIndex);
	sprintf(fileNameE, "%de.jpg", picIndex);
	sprintf(fileNameP, "%dp.jpg", picIndex);
	picIndex++;

	std::string str_raw = getLogDirectory() + fileNameR;
	std::string str_enh = getLogDirectory() + fileNameE;
	std::string str_pro = getLogDirectory() + fileNameP;
	
	std::unique_lock<std::mutex> lock(timpl->m_mutex);
	imagesToSave.push(std::tuple<std::string, cv::Mat>
		(str_raw, (cam == CAM_FRONT)?raw_front:raw_down));
	imagesToSave.push(std::tuple<std::string, cv::Mat>
		(str_enh, (cam == CAM_FRONT)?i_front:i_down));
	imagesToSave.push(std::tuple<std::string, cv::Mat>(str_pro, img.clone()));
	lock.unlock();
	
	lprintf("saving jpg at %\n", str_raw.c_str());
}

/*
 * This function should be used instead of calling a while(requestImage) loop.
 * It pauses for 5ms in between checking requestImage, greatly reducing CPU usage.
 */
void Vision::waitForImage() volatile
{
	while (requestImage)
	{
		Timer::sleep(5);
	}
}
