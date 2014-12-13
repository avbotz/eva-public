/*
 _                                         
| |__  _   _  ___  _   _   ___ _ __  _ __  
| '_ \| | | |/ _ \| | | | / __| '_ \| '_ \ 
| |_) | |_| | (_) | |_| || (__| |_) | |_) |
|_.__/ \__,_|\___/ \__, (_)___| .__/| .__/ 
                   |___/      |_|   |_|    
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
#include "buoy.hpp"

Buoy::Buoy(libconfig::Config *c) : VisionTask(c, CAM_FRONT),
	initialHeading(State::getProperty(State::currentHeading))
{
	conf->lookupValue("buoy.w_min_pixel_intensity", wMinPixelIntensity);
	conf->lookupValue("buoy.blob_min_area", blobMinArea);
	conf->lookupValue("buoy.blob_max_area", blobMaxArea);
	conf->lookupValue("buoy.depth", buoyDepth);

	currentBuoy = 0;
	
	for (int i = 0; i < 3; i++)
	{
		buoy_blob[i] = nullptr;
	}

	conf->lookupValue("buoy.backup_time", backupTime);
	conf->lookupValue("buoy.move_on_time", moveOnTime);
	conf->lookupValue("buoy.ram_area", ramArea);
	conf->lookupValue("buoy.ram_time", ramTime);
	
	conf->lookupValue("buoy.move_on_depth", moveOnDepth);
	conf->lookupValue("buoy.heading_constant", headingConstant);
	conf->lookupValue("buoy.depth_constant", depthConstant);
	conf->lookupValue("buoy.end_angle_offset", endAngleOffset);
	conf->lookupValue("buoy.depth_constant", rMinPixelIntensity);
	
	conf->lookupValue("buoy.h_threshold", hThreshold);
	conf->lookupValue("buoy.v_threshold", vThreshold);
	
	conf->lookupValue("buoy.wall_blob_height", wallBlobHeight);
	conf->lookupValue("buoy.wall_blob_distance_x", wallBlobDistanceX);
	conf->lookupValue("buoy.min_buoy_circularity", minBuoyCircularity);
	conf->lookupValue("buoy.min_buoy_density", minBuoyDensity);
	conf->lookupValue("buoy.glare_blob_width", glareBlobWidth);
	conf->lookupValue("buoy.glare_blob_height", glareBlobHeight);
	
	conf->lookupValue("buoy.rb", rb);
	conf->lookupValue("buoy.rg", rg);
	conf->lookupValue("buoy.gr", gr);
	conf->lookupValue("buoy.gb", gb);
	conf->lookupValue("buoy.blob_prop_ratio", blobPropRatio);
	conf->lookupValue("buoy.blob_prop_width", blobPropWidth);
	conf->lookupValue("buoy.blob_prop_height", blobPropHeight);
	conf->lookupValue("buoy.max_depth", maxDepth);
	
	stage = Init;

	haveWeHit[B_RED] = false;
	haveWeHit[B_GREEN] = false;
	haveWeHit[B_CHANGING] = false;
}

Buoy::~Buoy()
{
}

void Buoy::action()
{
	static bool weShouldResetTimer = false;
	switch (stage)
	{
		case Init:
			State::setProperty(State::desiredPower, 100);
			State::setProperty(State::desiredHeading, State::getProperty(State::currentHeading));
			State::setProperty(State::desiredDepth, buoyDepth);
			
			initializePan(initialHeading, 20);
			currentPanHeading = 0;
			timer.resetTimer();
			timeout.resetTimer();

			pause = 10000;
			weShouldResetTimer = true;

			initialHeading = State::getProperty(State::currentHeading);

			stage = Ram;
			return;
		case Ram:
			if (weShouldResetTimer)
			{
				timeout.resetTimer();
				weShouldResetTimer = false;
			}
			if (ram())
			{
				forceSaveFrame = true;
				stage = Backup;
			}
			return;
		case Backup:
			State::setProperty(State::desiredPower, 0);
			pause = backupTime;
			stage = ResetHeadingAndDepth;
			return;
		case ResetHeadingAndDepth:
			State::setProperty(State::desiredPower, 100);
			State::setProperty(State::desiredHeading, initialHeading);
			State::setProperty(State::desiredDepth, buoyDepth);
			pause = 5000;
			weShouldResetTimer = true;
			stage = Ram;
			return;
		case MoveOn:
			State::setProperty(State::desiredHeading, State::getEquivalentAngle(initialHeading + endAngleOffset));
			State::setProperty(State::desiredDepth, moveOnDepth);
			State::setProperty(State::desiredPower, 200);
			stage = Finish;
			pause = moveOnTime;
			return;
		case Finish:
			State::setProperty(State::desiredPower, 100);
			State::setProperty(State::desiredDepth, buoyDepth);
			pause = 5000;
			isCompleted = true;
			return;
		default:
			return;
	}
}

/*
 * find buoy and ram it
 */
bool Buoy::ram()
{
	processImage();
	Blob* b = nullptr;

	for (int i = 0; i < num_blobs; i++)
	{
		if (buoy_blob[i]->getColor() == B_RED && !haveWeHit[B_RED])
		{
			b = buoy_blob[i];
			break;
		}
		if (buoy_blob[i]->getColor() == B_GREEN && !haveWeHit[B_GREEN])
		{
			b = buoy_blob[i];
			break;
		}
		if (buoy_blob[i]->getColor() == B_CHANGING && !haveWeHit[B_CHANGING])
		{
			b = buoy_blob[i];
			break;
		}
	}

	if(haveWeHit[B_RED] && haveWeHit[B_GREEN] && haveWeHit[B_CHANGING])
	{
		stage = MoveOn;
		return false;
	}

	if (b == nullptr)
	{
		if(timeout.getCurrentTime() > 10000)
		{
			stage = MoveOn;
		}
		return false;
	}
	else
	{
		timeout.resetTimer();
	}

	// if close enough, go forwards a bit then move on
	if (b->area >= ramArea)
	{
		State::setProperty(State::desiredPower, 200);
		pause = ramTime;
		haveWeHit[b->getColor()] = true;
		return true;
	}
	
	// go to buoy
	float dist = (float)(b->getCentroid().x - pic_width / 2) / pic_width;
	float vDist = (float)(b->getCentroid().y - pic_height / 2) / pic_height;
	if (abs(dist) < hThreshold && abs(vDist) < vThreshold)
	{
		State::setProperty(State::desiredPower, 200);
	}
	else
	{
		State::setProperty(State::desiredPower, 100);
	}
	State::setProperty(State::desiredHeading, State::getEquivalentAngle(State::getProperty(State::currentHeading) + dist * headingConstant));
	State::setProperty(State::desiredDepth, State::getProperty(State::currentDepth) + vDist * depthConstant);
	lprintf("ram: hDist = %, vDist = %\n", dist, vDist);
	
	return false;
}

bool Buoy::check()
{
	return true;
}

/*
 * Processes a frame from Vision. Records
 * the number of buoys it finds
 * Looks for buoys of all colors
 */
void Buoy::processImage()
{
	// obtain images from the front camera.
	cv::Mat image = State::getImage(State::imageFront);

	// reset processedImage and define the image size
	memset(processedImage.ptr(), 0, processedImage.rows*processedImage.cols*3);
	int image_size = image.rows*image.cols;

	cv::Mat imgR, imgG, imgW;
	imgR.create(pic_height, pic_width, CV_8UC1);
	imgG.create(pic_height, pic_width, CV_8UC1);
	imgW.create(pic_height, pic_width, CV_8UC1);

	unsigned char *rPtr = imgR.ptr(), *gPtr = imgG.ptr(), *wPtr = imgW.ptr();

	const unsigned char* ip = image.ptr();
	
	// get average intensity for each color channel (for red thresholding)
	int totalB = 0, totalG = 0, totalR = 0;
	for(int i = 0; i < image_size; i++)
	{
		totalB += ip[3*i];
		totalG += ip[3*i+1];
		totalR += ip[3*i+2];
	}
	totalB /= image_size;
	totalG /= image_size;
	totalR /= image_size;

	// threshold colors (red, green, white)
	auto threshold_colors = [=](int start, int end)
	{
		for (int i = start; i < end; i++)
		{
			int b = ip[3*i], g = ip[3*i+1], r = (int)ip[3*i+2];
			
			int blue_avg_val = b - totalB;
			int green_avg_val = g - totalG;
			int red_avg_val = r - totalR;

			bool checkR = false;
			if(red_avg_val > blue_avg_val+rb && red_avg_val > green_avg_val+rg)
			{
				checkR = true;
			}

			int red_val = 2*r - 2*b;
			int wht_val = r + g + b - 2*MAX3(r,g,b);

			if (red_val > 4*rMinPixelIntensity/3)
			{
				wht_val -= red_val;
			}

			rPtr[i] = checkR ? 255 : 0;
			
			gPtr[i] = (g > b + gb && g > r + gr) * 255;
			
			wPtr[i] = (wht_val < wMinPixelIntensity) ? 0 : ((wht_val > 255) ? 255 : wht_val);

			processedImage.ptr()[3*i+2] = rPtr[i];
			processedImage.ptr()[3*i+1] = gPtr[i];
			processedImage.ptr()[3*i] = wPtr[i];
		}
	};

	// multithread the thresholding.
	pool.add_task(threshold_colors,              0,   image_size/4);
	pool.add_task(threshold_colors,   image_size/4,   image_size/2);
	pool.add_task(threshold_colors,   image_size/2, 3*image_size/4);
	pool.add_task(threshold_colors, 3*image_size/4,	    image_size);

	pool.wait();

	std::vector<Blob*> blobR, blobG, blobW;
	
	// find blobs
	auto detect_blobs = [&](cv::Mat src, int color, std::vector<Blob*>* blobs)
	{
		*blobs = blob_detection(src, color, 3);
	};

	// multithread blob detection
	pool.add_task(detect_blobs, imgR,      B_RED, &blobR);
	pool.add_task(detect_blobs, imgG,    B_GREEN, &blobG);
	pool.add_task(detect_blobs, imgW, B_CHANGING, &blobW);
	
	pool.wait();

	int num_red = 0, num_green = 0, num_white = 0;
	float ratio = 0;
	
	// no check if each blob is a buoy
	num_blobs = 0;
	// process red blobs
	for(unsigned int i = 0; i < blobR.size(); i++)
	{
		// if it is too small/large, skip it
		if(blobR[i]->area < blobMinArea || blobR[i]->area > blobMaxArea)
		{
			delete blobR[i];
			continue;
		}

		// if it is tall and to the side, it is probably the wall
		if (blobR[i]->max_y - blobR[i]->min_y > wallBlobHeight * pic_height && MAX(blobR[i]->max_x - pic_width / 2, pic_width/2 - blobR[i]->min_x) < wallBlobDistanceX * pic_width)
		{
			delete blobR[i];
			continue;
		}

		// if it is a wide, high blob, it is probably glare
		if (blobR[i]->max_x - blobR[i]->min_x > glareBlobWidth * pic_width && blobR[i]->min_y < (0.5 - glareBlobHeight) * pic_height)
		{
			lprintf("% %\n", blobR[i]->max_x-blobR[i]->min_x, glareBlobWidth);
			delete blobR[i];
			continue;
		}

		// if it is a wide, low blob, it is probably noise
		if (blobR[i]->max_x - blobR[i]->min_x > glareBlobWidth * pic_width && blobR[i]->max_y > (0.5 + glareBlobHeight) * pic_height)
		{
			lprintf("% %\n", blobR[i]->max_x-blobR[i]->min_x, glareBlobWidth);
			delete blobR[i];
			continue;
		}

		// if it is disproportional and the height or width exceeds a certain percentage of the image, it is probably noise
		ratio = (float)(blobR[i]->max_x-blobR[i]->min_x+1) / (blobR[i]->max_y-blobR[i]->min_y + 1);
		if((ratio > blobPropRatio || (1/ratio) > blobPropRatio) && (blobR[i]->max_x-blobR[i]->min_x > blobPropWidth*pic_width || blobR[i]->max_y-blobR[i]->min_y > blobPropHeight*pic_height))
		{
			delete blobR[i];
			continue;
		}

		// if it isn't dense enough, it is probably noise
		if ((float)blobR[i]->area / blobR[i]->getConvexHullArea() < minBuoyDensity)
		{
			delete blobR[i];
			continue;
		}

		// if it is too low, it is probably the floor
		if (State::getProperty(State::currentDepth) > maxDepth && blobR[i]->getCentroid().y > pic_height / 2)
		{
			delete blobR[i];
			continue;
		}

		// check for blob circularity. If it fails, remove it
		if (blobR[i]->getHullCircularity() > minBuoyCircularity && num_red != 1)
		{
			buoy_blob[num_blobs] = new Blob(*blobR[i]);
			num_blobs++;
			num_red++;
		}
		else
		{
			delete blobR[i];
		}
	}
	// process green blobs
	for(unsigned int i = 0; i < blobG.size(); i++)
	{
		// if it is too small/large, skip it
		if(blobG[i]->area < blobMinArea || blobG[i]->area > blobMaxArea)
		{
			delete blobG[i];
			continue;
		}

		// if it is tall and to the side, it is probably the wall
		if (blobG[i]->max_y - blobG[i]->min_y > wallBlobHeight && MAX(blobG[i]->max_x - pic_width / 2, pic_width/2 - blobG[i]->min_x) < wallBlobDistanceX)
		{
			lprintf("% %\n", blobG[i]->max_x-blobG[i]->min_x, wallBlobHeight);
			delete blobG[i];
			continue;
		}

		// if it is a wide, high blob, it is probably glare
		if (blobG[i]->max_x - blobG[i]->min_x > glareBlobWidth * pic_width && blobG[i]->min_y < (0.5 - glareBlobHeight) * pic_height)
		{
			lprintf("% %\n", blobG[i]->max_x-blobG[i]->min_x, glareBlobWidth);
			delete blobG[i];
			continue;
		}

		// if it is a wide, low blob, it is probably noise
		if (blobG[i]->max_x - blobG[i]->min_x > glareBlobWidth * pic_width && blobG[i]->max_y > (0.5 + glareBlobHeight) * pic_height)
		{
			lprintf("% %\n", blobG[i]->max_x-blobG[i]->min_x, glareBlobWidth);
			delete blobG[i];
			continue;
		}

		// if it is disproportional and the height or width exceeds a certain percentage of the image, it is probably noise.
		ratio = (float)(blobG[i]->max_x-blobG[i]->min_x+1) / (blobG[i]->max_y-blobG[i]->min_y);
		if((ratio > blobPropRatio || (1/ratio) > blobPropRatio) && (blobG[i]->max_x-blobG[i]->min_x > blobPropWidth*pic_width || blobG[i]->max_y-blobG[i]->min_y > blobPropHeight*pic_height))
		{
			lprintf("% %\n", blobG[i]->max_x-blobG[i]->min_x, blobPropRatio);
			delete blobG[i];
			continue;
		}

		// if it isn't dense enough, it is probably noise
		if ((float)blobG[i]->area / blobG[i]->getConvexHullArea() < minBuoyDensity)
		{
			delete blobG[i];
			continue;
		}

		// if it is too low, it is probably the floor
		if (State::getProperty(State::currentDepth) > maxDepth && blobG[i]->getCentroid().y > pic_height / 2)
		{
			delete blobG[i];
			continue;
		}

		// check for blob circularity; ff it fails, save it for later
		if (blobG[i]->getHullCircularity() > minBuoyCircularity && num_green != 1)
		{

			buoy_blob[num_blobs] = new Blob(*blobG[i]);
			num_blobs++;
			num_green++;

			continue;
		}

		// if we have seen a red blob and haven't got a circular green one yet,
		// check if this green blob is near the same y-axis as the red one
		// (as they are around the same height)
		// it could be the green buoy, but that is unlikely
		else if(num_red == 1 && num_green != 1)
		{
			if(buoy_blob[0]->getCentroid().y < blobG[i]->getCentroid().y+(pic_width/8)
				&& buoy_blob[0]->getCentroid().y > blobG[i]->getCentroid().y-(pic_width/8))
			{
				buoy_blob[num_blobs] = new Blob(*blobG[i]);
				num_blobs++;
				num_green++;

				continue;
			}
		}
		else
		{
			delete blobG[i];
		}
	}

	// process white blobs
	for(unsigned int i = 0; i < blobW.size(); i++)
	{
		// if it is too small/large, skip it
		if(blobW[i]->area < blobMinArea || blobW[i]->area > blobMaxArea)
		{
			delete blobW[i];
			continue;
		}

		// if it isn't dense enough, it is probably noise
		if ((float)blobW[i]->area / blobW[i]->getConvexHullArea() < minBuoyDensity)
		{
			delete blobW[i];
			continue;
		}

		// if it is tall and to the side, it is probably the wall
		if (blobW[i]->max_y - blobW[i]->min_y > wallBlobHeight * pic_height && MAX(blobW[i]->max_x - pic_width / 2, pic_width/2 - blobW[i]->min_x) < wallBlobDistanceX * pic_width)
		{
			delete blobW[i];
			continue;
		}

		// if it is a wide, high blob, it is probably glare
		if (blobW[i]->max_x - blobW[i]->min_x > glareBlobWidth && blobW[i]->min_y < (0.5 - glareBlobHeight) * pic_height)
		{
			delete blobW[i];
			continue;
		}

		// if it is a wide, low blob, it is probably noise
		if (blobW[i]->max_x - blobW[i]->min_x > glareBlobWidth * pic_width && blobW[i]->max_y > (0.5 + glareBlobHeight) * pic_height)
		{
			delete blobW[i];
			continue;
		}

		// if it is disproportional and the height or width exceeds a certain percentage of the image, it is probably noise
		ratio = (float)(blobW[i]->max_x-blobW[i]->min_x+1) / (blobW[i]->max_y-blobW[i]->min_y);
		if((ratio > blobPropRatio || (1/ratio) > blobPropRatio) && (blobW[i]->max_x-blobW[i]->min_x > blobPropWidth*pic_width || blobW[i]->max_y-blobW[i]->min_y > blobPropHeight*pic_height))
		{
			delete blobW[i];
			continue;
		}

		// check the 'rectangularity' of the blob
		cv::RotatedRect rect = cv::minAreaRect(blobW[i]->convexHull);
		float angle = (abs(rect.angle) > 45) ? (abs(rect.angle) - 90) : abs(rect.angle);
		float rRatio = (float)rect.size.height / (float)rect.size.width;
		if (angle > 10 && (rRatio < .9 || rRatio > 1.11111))
		{
			delete blobW[i];
			continue;
		}

		// if it is too low, it is probably the floor
		if (State::getProperty(State::currentDepth) > maxDepth && blobW[i]->getCentroid().y > pic_height / 2)
		{
			delete blobW[i];
			continue;
		}

		// white thresholding is picking up white from the red/green buoy,
		// so don't include it
		bool deletedWhite = false;
		for(unsigned int j = 0; j < num_blobs; j++)
		{
			if(blobW[i]->getCentroid().x < (buoy_blob[j]->getCentroid().x+(pic_width/16))
				&& blobW[i]->getCentroid().x > (buoy_blob[j]->getCentroid().x-(pic_width/16)))
			{
				deletedWhite = true;
				break;
			}
		}

		if(deletedWhite)
		{
			delete blobW[i];
			continue;
		}

		// if we've seen no white blobs yet
		if (num_white < 1)
		{
			buoy_blob[num_blobs] = new Blob(*blobW[i]);

			num_blobs++;
			num_white++;

			// we don't need the other white blobs
			break;
		}
	}

	// if we've got less than the three blobs, fill the remainder with nullptr's
	// (to get rid of blobs from the previous frame)
	for (int i = num_red+num_white+num_green; i < 3; i++)
	{
		buoy_blob[i] = nullptr;
	}

	lprintf("% % % %\n", num_blobs, num_red, num_green, num_white);
	// draw identified blobs
	for (unsigned int i = 0; i < num_blobs; i++)
	{
		Blob* b = buoy_blob[i];

		cv::Scalar color;

		switch (b->getColor())
		{
			case B_RED: color = cv::Scalar(0, 0, 255); break;
			case B_GREEN: color = cv::Scalar(0, 255, 0); break;
			case B_CHANGING: color = cv::Scalar(255, 0, 0); break;
		}
		b->drawBlob(processedImage, color);
	}

	// write num_blobs to the processedImage
	char num_str[3];
	sprintf(num_str, "%d", num_blobs);
	cv::putText(processedImage, num_str, cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,200,200), 1, CV_AA);
	return;
}
