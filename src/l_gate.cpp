/*
 _                 _                         
| |     __ _  __ _| |_ ___   ___ _ __  _ __  
| |    / _` |/ _` | __/ _ \ / __| '_ \| '_ \ 
| |   | (_| | (_| | ||  __/| (__| |_) | |_) |
|_|____\__, |\__,_|\__\___(_)___| .__/| .__/ 
 |_____|___/                    |_|   |_|    
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
#include "l_gate.hpp"

L_gate::L_gate(libconfig::Config *c) : VisionTask(c, CAM_FRONT),
	img(pic_height, pic_width, CV_8UC1)
{
	conf->lookupValue("l_gate.depth", gateDepth);

	conf->lookupValue("l_gate.init_power", initPower);
	conf->lookupValue("l_gate.approach_power", approachPower);
	
	conf->lookupValue("l_gate.approach_wait_time", approachWaitTime);
	conf->lookupValue("l_gate.adjust_wait_time", adjustWaitTime);
	conf->lookupValue("l_gate.adjust_timeout", advanceTime);
	
	conf->lookupValue("l_gate.turn_heading", turnHeading);
	
	conf->lookupValue("l_gate.check_green", checkGreen);

	conf->lookupValue("l_gate.attempt_circle", attemptCircle);
	conf->lookupValue("l_gate.circle_power1", circlePower1);
	conf->lookupValue("l_gate.circle_power2", circlePower2);
	conf->lookupValue("l_gate.circle_increment", circleIncrement);
	conf->lookupValue("l_gate.circle_turn_time", circleTurnTime);
	conf->lookupValue("l_gate.circle_move_time", circleMoveTime);

	conf->lookupValue("l_gate.rb_pixel_intensity", rbPixelIntensity);
	conf->lookupValue("l_gate.rg_pixel_intensity", rgPixelIntensity);
	conf->lookupValue("l_gate.r_min_blob_area", rMinBlobArea);
	conf->lookupValue("l_gate.gr_pixel_intensity", grPixelIntensity);
	conf->lookupValue("l_gate.gb_pixel_intensity", gbPixelIntensity);
	conf->lookupValue("l_gate.g_min_blob_area", gMinBlobArea);
	conf->lookupValue("l_gate.w_min_pixel_intensity", wMinPixelIntensity);
	
	conf->lookupValue("l_gate.end_power", endPower);
	conf->lookupValue("l_gate.end_pause", endPause);
	conf->lookupValue("l_gate.end_angle_offset", endAngleOffset);
	
	conf->lookupValue("l_gate.box_ratio", boxRatio);

	// set advanceTime (as a timeout) for the ADJUST stage.
	setAdvRegTime(advanceTime, 0);

	totalTurnAngle = 0;
}

L_gate::~L_gate()
{

}

void L_gate::action()
{
	switch (stage) {
	case Init:
		// initialize pan
		initializePan(State::getProperty(State::currentHeading), 15);
		State::setProperty(State::desiredDepth, gateDepth);
		currentPanHeading = 0;
		timer.resetTimer();
		
		// set initial power
		State::setProperty(State::desiredPower, initPower);

		initialAngle = State::getProperty(State::currentHeading);
		
		// proceed to next stage (no need for return)
		stage = Search;
	case Search:
		search();
		return;
	case Approach:
		approach();
		return;
	case Adjust:
		adjust();
		return;
	case CircleCheck:
		circleCheck();
		return;
	case CircleTurn:
		State::setProperty(State::desiredPower, 100);
		State::setProperty(State::desiredHeading, State::getEquivalentAngle(State::getProperty(State::currentHeading) + circleIncrement));
		totalTurnAngle += abs(circleIncrement);
		pause = circleTurnTime;
		stage = CircleMove;
		return;
	case CircleMove:
		if (totalTurnAngle >= 270)
		{
			State::setProperty(State::desiredPower, circlePower2);
		}
		else
		{
			State::setProperty(State::desiredPower, circlePower1);
		}
		pause = circleMoveTime;
		if (totalTurnAngle >= 360)
		{
			State::setProperty(State::desiredHeading, State::getEquivalentAngle(initialAngle + endAngleOffset));
			State::setProperty(State::desiredPower, endPower);
			pause = endPause;
			stage = Done;
		}
		else
		{
			stage = CircleTurn;
		}
		return;
	case Done:
		isCompleted = true;
		return;
	}
}

/*
 * checks if L_gate is avaliable
 */
bool L_gate::check()
{
	return true;
}

/*
 * searches for a satisfactory red blob
 */
void L_gate::search()
{
	// pan heading timed out, switch to the next one.
	if (timer.getCurrentTime() > 4000)
	{
		goToNextPanHeading();
		timer.resetTimer();
	}

	// if processImage() returns true, we can start the approach.
	if(processImage())
	{
		stage = Approach;
	}

	return;
}

/*
 * Approaches the red blob with an offset.
 * Also, adjust for depth.
 */
void L_gate::approach()
{
	// process new frame
	processImage();
	
	// stop moving to adjust for depth
	State::setProperty(State::desiredPower, 100);

	if(blobCenter.y > (pic_height/2 + pic_height/8))
	{
		// we're too high, so go down
		State::setProperty(State::desiredDepth, State::getProperty(State::currentDepth) + 5);
		return;
	}
	else if(blobCenter.y < (pic_height/2 - pic_height/8))
	{
		// we're too low, so go up
		if(State::getProperty(State::currentDepth) < 30)
		{
			// don't breach.
			State::setProperty(State::desiredDepth, 30);
		}
		else
		{
			State::setProperty(State::desiredDepth, State::getProperty(State::currentDepth) - 5);
			return;
		}
	}

	// set approach power
	State::setProperty(State::desiredPower, approachPower);

	// set desired heading to the blob plus any offset defined in competition/simulation.conf
	State::setProperty(State::desiredHeading, State::getEquivalentAngle(getPointHeading(blobCenter) + turnHeading));

	// pause for the time set in approachWaitTime
	pause = approachWaitTime;
	// pave a frame, just in case we did that too fast for snapshot_time
	forceSaveFrame = true;
	stage = Adjust;

	return;
}

/*
 * Adjust to the green parts of the task
 */
void L_gate::adjust()
{
	// obtain image in cv::Mat format
	cv::Mat image = State::getImage(State::imageFront);
	int image_size = image.rows*image.cols;
	const unsigned char* ip = image.ptr();

	// reset processedImage
	memset(processedImage.ptr(), 0, 3*processedImage.rows * processedImage.cols);

	// multi-thread green thresholding
	auto threshold_color = [=](int start, int end)
	{
		for (int i = start; i < end; i++)
		{
			int b = ip[3*i], g = ip[3*i+1], r = (int)ip[3*i+2];
			img.ptr()[i] = (g > b + gbPixelIntensity && g > r + grPixelIntensity) * 255;

			processedImage.ptr()[3*i+1] = img.ptr()[i];
		}
	};

	pool.add_task(threshold_color,              0,   image_size/4);
	pool.add_task(threshold_color,   image_size/4,   image_size/2);
	pool.add_task(threshold_color,   image_size/2, 3*image_size/4);
	pool.add_task(threshold_color, 3*image_size/4,     image_size);

	pool.wait();

	// runs blob detection and stores results in a std::vector of blobs
	std::vector<Blob*> blobs = blob_detection(img, 0, 1);

	// checks if there are any green blobs; if not, give up
	if(blobs.size() == 0 || blobs.size() < 3)
	{
		blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());
		checkAdvance();
		return;
	}

	// purge those small blobs, leaving only the largest 3
	if(blobs.size() > 3)
	{
		blobs.erase(std::remove_if(blobs.begin() + 3, blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());
	}

	// draws blobs
	for(int i = 0; i < blobs.size(); i++)
	{
		blobs[i]->drawBlob(processedImage, cv::Scalar(0, 255, 0));
	}

	// check if the largest's area meets the requisites; else, give up
	if(blobs[0]->area < gMinBlobArea)
	{
		blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());
		checkAdvance();
		return;
	}

	// check if those blobs are the ones we are looking for
	// if the large blob is centered between the smaller ones, they're probably the right ones
	int averageCenter = (blobs[1]->getCentroid(0).x + blobs[2]->getCentroid(0).x)/2;
	if(averageCenter < (blobs[0]->getCentroid(0).x + pic_width/16) && averageCenter > (blobs[0]->getCentroid(0).x - pic_width/16))
	{
		// if turnHeading is positive (or zero), head right; else, go left
		// aiming for the middle-right or middle-left
		int newHeading;
		if(turnHeading >= 0)
		{
			int rightBlob = (blobs[1]->getCentroid(0).x > blobs[2]->getCentroid(0).x) ? 1 : 2;
			newHeading = (blobs[rightBlob]->getCentroid(0).x + blobs[0]->getCentroid(0).x)/2;
		}
		else
		{
			int leftBlob = (blobs[1]->getCentroid(0).x > blobs[2]->getCentroid(0).x) ? 2 : 1;
			newHeading = (blobs[leftBlob]->getCentroid(0).x + blobs[0]->getCentroid(0).x)/2;
		}

		State::setProperty(State::desiredHeading, getPointHeading(cv::Point(newHeading, 0)));
		blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());

		// if we are checking for green blobs before circling or if we are
		// not attempting the circle, we want to pause for a bit
		if(!checkGreen || !attemptCircle)
		{
			pause = adjustWaitTime;
		}

		// save frame just in case we're too fast, since we want some helpful log data
		forceSaveFrame = true;
		if (attemptCircle == true)
		{
			// change hardware request to down, given that CIRCLECHECK looks down for green blobs
			hardware_request = CAM_DOWN;
			stage = CircleCheck;
		}
		else
		{
			stage = Done;
		}
	}
	else
	{
		blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());
		checkAdvance();
	}
}

/*
 * Times out the ADJUST stage if we waited longer than adjustTime with no results.
 */
void L_gate::checkAdvance()
{
	if(nextStage(true, false) == CircleCheck)
	{
		if((!checkGreen || !attemptCircle) && adjustWaitTime > advanceTime)
		{
			pause = adjustWaitTime - advanceTime;
		}
		if(attemptCircle == false)
		{
			stage = Done;
		}
		else
		{
			hardware_request = CAM_DOWN;
			stage = CircleCheck;
		}
	}
}

/* 
 * Checks for green blobs on the down camera, marking the beginning of the circle.
 */
void L_gate::circleCheck()
{
	// if we don't want to check for green, move on
	if(!checkGreen)
	{
		stage = CircleTurn;
		return;
	}

	// obtain image in cv::Mat format
	cv::Mat image;
	cvtColor(State::getImage(State::imageDown), image, CV_RGB2GRAY);
	int image_size = image.rows*image.cols;
	const unsigned char* ip = image.ptr();

	// multi-thread brightness thresholding, again
	auto threshold_color = [&](int start, int end)
	{
		for (int i = start; i < end; i++)
		{
			img.ptr()[i] = (image.ptr()[i] < wMinPixelIntensity) ? 0 : 255;
		}
	};

	pool.add_task(threshold_color,              0,   image_size/4);
	pool.add_task(threshold_color,   image_size/4,   image_size/2);
	pool.add_task(threshold_color,   image_size/2, 3*image_size/4);
	pool.add_task(threshold_color, 3*image_size/4,     image_size);

	pool.wait();

	processedImage = image;

	// runs blob detection and stores results in a std::vector of blobs
	std::vector<Blob*> blobs = blob_detection(img, 0, 1);

	// checks if there are any green blobs
	if(blobs.size() == 0)
	{
		lprintf("circleCheck: can't find anything yet\n");
		blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());
		return;
	}

	for (Blob* b : blobs)
	{
		// check if the largest blob is a bar-like object going across the screen horizontally
		cv::RotatedRect rect = cv::minAreaRect(blobs[0]->convexHull);
		float ratio = (float)rect.size.height / rect.size.width;
		lprintf("circleCheck: PVC ratio is %\n", ratio);
		if (b->area < gMinBlobArea && (ratio < boxRatio || ratio > 1 / boxRatio))
		{
			std::string ratioString = "Ratio: " + std::to_string(ratio);
			cv::putText(processedImage, ratioString, b->getCentroid(), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,200,200), 1, CV_AA);
			b->drawBlob(processedImage, cv::Scalar(255, 255, 255));
			// if it is close enough, move on
			if(b->getCentroid(0).y > (pic_height/2 - pic_height/8))
			{
				forceSaveFrame = true;
				stage = CircleTurn;
			}
		}
	}

	blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());
	return;
}

/*
 * Threshold image for red and get the largest blob's centroid.
 */
bool L_gate::processImage(bool check)
{
	// obtain image in cv::Mat format
	cv::Mat image = State::getImage(State::imageFront);
	int image_size = image.rows*image.cols;
	const unsigned char* ip = image.ptr();

	// reset processedImage
	if(check != true)
	{
		memset(processedImage.ptr(), 0, 3*processedImage.rows * processedImage.cols);
	}

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

	// multi-thread red thresholding
	auto threshold_color = [=](int start, int end)
	{
		for (int i = start; i < end; i++)
		{
			int b = ip[3*i], g = ip[3*i+1], r = (int)ip[3*i+2];
			
			int blue_avg_val = b - totalB;
			int green_avg_val = g - totalG;
			int red_avg_val = r - totalR;
			
			bool checkR = false;
			if(red_avg_val > blue_avg_val+rbPixelIntensity && red_avg_val > green_avg_val+rgPixelIntensity)
			{
				checkR = true;
			}

			img.ptr()[i] = checkR ? 255 : 0;
			
			if(check != true)
			{
				processedImage.ptr()[3*i+2] = img.ptr()[i];
			}
		}
	};

	pool.add_task(threshold_color,              0,   image_size/4);
	pool.add_task(threshold_color,   image_size/4,   image_size/2);
	pool.add_task(threshold_color,   image_size/2, 3*image_size/4);
	pool.add_task(threshold_color, 3*image_size/4,     image_size);

	pool.wait();

	// runs blob detection and stores results in a std::vector of blobs
	std::vector<Blob*> blobs = blob_detection(img, 0, 1);

	// checks if there are any red blobs
	if(blobs.size() == 0)
	{
		return false;
	}

	// checks if the largest blob meets the size requirement
	for(unsigned int i = 0; i < blobs.size(); i++)
	{
		if(blobs[i]->area < rMinBlobArea)
		{
			blobs.erase(std::remove_if(blobs.begin() + i, blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());
		}
	}
	
	// save inital processedImage; no need if it's just a check
	if(check != true)
	{
		for(int i = 0; i < blobs.size(); i++)
		{
			blobs[i]->drawBlob(processedImage, cv::Scalar(0, 0, 255));
		}
	}

	int desiredBlob = -1;

	for(unsigned int i = 0; i < blobs.size(); i++)
	{
		// check if the blob represents the cylindrical thing we're looking for or not
		cv::RotatedRect rect = cv::minAreaRect(blobs[i]->convexHull);
		float angle = (abs(rect.angle) > 45) ? (abs(rect.angle) - 90) : abs(rect.angle);
		float ratio = (float)rect.size.height / (float)rect.size.width;
		if((angle > 10) && (ratio < 5))
		{
			continue;
		}
		else
		{
			desiredBlob = i;
		}
	}

	if(desiredBlob != -1)
	{
		// if (check == true), simply return true as it's just a check
		if(check == true)
		{
			blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());
			return true;
		}


		// set blobCenter equal to the center of the red blob
		blobCenter = blobs[desiredBlob]->getCentroid();

		// save centroid data on processedImage
		int xCenter = blobs[desiredBlob]->getCentroid(0).x;
		int yCenter = blobs[desiredBlob]->getCentroid(0).y;
		std::string coordinates = "Red Centroid: (" + std::to_string(xCenter) + ", " + std::to_string(yCenter) + ")";
		cv::putText(processedImage, coordinates, cvPoint(20, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,200,200), 1, CV_AA);

		blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());
		return true;
	}
	else
	{
		blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());
		return false;
	}
}
