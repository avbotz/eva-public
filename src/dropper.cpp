/*
     _                                                 
  __| |_ __ ___  _ __  _ __   ___ _ __ ___ _ __  _ __  
 / _` | '__/ _ \| '_ \| '_ \ / _ \ '__/ __| '_ \| '_ \ 
| (_| | | | (_) | |_) | |_) |  __/ | | (__| |_) | |_) |
 \__,_|_|  \___/| .__/| .__/ \___|_|(_)___| .__/| .__/ 
                |_|   |_|                 |_|   |_|    
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
#include "dropper.hpp"
#include "blob_detection.hpp"
#include <math.h>
#include <string>
#include <unistd.h>

Dropper::Dropper(libconfig::Config *conf) : VisionTask(conf, CAM_DOWN),
	currentBin(0),
	image_size(pic_height * pic_width)
{
	// for navigation
	conf->lookupValue("dropper.primary_depth", primaryDepth);

	conf->lookupValue("dropper.min_bin_area", minBinArea);
	conf->lookupValue("dropper.max_bin_area", maxBinArea);
	conf->lookupValue("dropper.min_border_area", minBorderArea);
	conf->lookupValue("dropper.max_border_area", maxBorderArea);
	conf->lookupValue("dropper.min_bin_border_dist", minBinBorderDist);
	conf->lookupValue("dropper.max_bin_border_dist", maxBinBorderDist);
	conf->lookupValue("dropper.density", density);

	pic_diag = sqrt(pow(pic_height, 2) + pow(pic_width, 2));
	conf->lookupValue("dropper.dist_all_bins", distAllBins);
	conf->lookupValue("dropper.dist_bin", distBin);

	conf->lookupValue("dropper.min_thres_border", minThresBorder);
	conf->lookupValue("dropper.max_thres_bin", maxThresBin);
	conf->lookupValue("dropper.aligned", aligned);
	
	conf->lookupValue("dropper.go_to_bin_power", goToBinPower);
	conf->lookupValue("dropper.go_to_bin_time", goToBinTime);
	conf->lookupValue("dropper.go_to_bin_angle", goToBinAngle);
	
	conf->lookupValue("dropper.adjust_heading_time", adjustHeadingTime);
	
	// for image comparison
	conf->lookupValue("dropper.img0", imgFile[0]);
	conf->lookupValue("dropper.img1", imgFile[1]);
	conf->lookupValue("dropper.img2", imgFile[2]);
	conf->lookupValue("dropper.img3", imgFile[3]);
	conf->lookupValue("dropper.canthres", canthres);
	conf->lookupValue("dropper.canratio", canratio);
	conf->lookupValue("dropper.linethres", linethres);
	
	// to decide if we should drop
	conf->lookupValue("dropper.accuracy_threshold", accuracyThreshold);
	conf->lookupValue("dropper.is_valid0", isValid[0]);
	conf->lookupValue("dropper.is_valid1", isValid[1]);
	conf->lookupValue("dropper.is_valid2", isValid[2]);
	conf->lookupValue("dropper.is_valid3", isValid[3]);
	conf->lookupValue("dropper.bin_position0", binPosition[0]);
	conf->lookupValue("dropper.bin_position1", binPosition[1]);
	conf->lookupValue("dropper.bin_position2", binPosition[2]);
	conf->lookupValue("dropper.bin_position3", binPosition[3]);
	conf->lookupValue("dropper.use_image_detection", useImageDetection);
	
	conf->lookupValue("dropper.camera_offset", cameraOffset);
	
	conf->lookupValue("dropper.end_angle_offset", endAngleOffset);
	conf->lookupValue("dropper.end_pause_time", endPauseTime);
	
	conf->lookupValue("dropper.search_power", searchPower);
	conf->lookupValue("dropper.approach_power", approachPower);
	conf->lookupValue("dropper.slow_approach_multiplier", slowApproachMultiplier);
	
	stage = Init;
	currentBin = 0;

	labelStage("Go to bins");
	labelStage("Finding next bin");
	labelStage("Approaching next bin");
	labelStage("Centering over bin");
	labelStage("Checking bin image");
	labelStage("Turning");
	labelStage("Going forwards");
	labelStage("Finished");
}

Dropper::~Dropper()
{

}

bool Dropper::check()
{
	return true;
}

void Dropper::action() 
{
	switch(stage)
	{
		case Init:
		{
			// store the initial heading for later
			// and set the initial power and depth 
			initialHeading = State::getProperty(State::currentHeading);
			State::setProperty(State::desiredPower, 100);
			State::setProperty(State::desiredHeading, State::getProperty(State::currentHeading));
			State::setProperty(State::desiredDepth, primaryDepth);
			stage = GoToBins;
			break;
		}
		case GoToBins:
		{
			// center the sub on the bins
			// if the sub is centered, move on
			if (goToBins())
			{
				forceSaveFrame = true;
				stage = CenterOverBin;
			}
			return;
		}
		case FindNextBin:
		{
			// find the next bin and go to it
			if (findNextBin())
			{
				forceSaveFrame = true;
				stage = ApproachBin;
			}
			return;
		}
		case ApproachBin:
		{
			pause = goToBinTime;
			State::setProperty(State::desiredPower, goToBinPower);
			stage = CenterOverBin;
			return;
		}
		case CenterOverBin:
		{
			// center the sub on the bin
			if (centerOverBin())
			{
				forceSaveFrame = true;
				stage = DetectAndDrop;
			}
			return;
		}
		case DetectAndDrop:
		{
			DetectedImage imageStatus;
			if (useImageDetection)
			{
				// try to detect the image
				imageStatus = detectImage();
			}
			else
			{
				if (isValid[binPosition[currentBin]])
				{
					imageStatus = DetectedImage::Drop;
				}
				else
				{
					imageStatus = DetectedImage::DontDrop;
				}
			}
			// if the image is not accurately identified, recenter and retry
			if (imageStatus == DetectedImage::Inaccurate)
			{
				stage = CenterOverBin;
			}
			else
			{
				// if the image is accurately identified as
				// something we want to drop on, drop
				// if we are running out of bins, drop

				if (imageStatus == DetectedImage::Drop)
				{
					State::setProperty(State::desiredPower, 100);
					State::setProperty(State::desiredHeading, State::getProperty(State::currentHeading));
					State::setProperty(State::dropperState, State::getProperty(State::dropperState) + 1);
					lprintf("Dropping.\n");
					pause = 1000;
				}
				// if the image is accurately identified as
				// something we don't want to drop on, do nothing
				else if (imageStatus == DetectedImage::DontDrop)
				{
				}
				// if this is the last bin or have dropped 2, end task
				if (currentBin == 3 || State::getProperty(State::dropperState) == 2)
				{
					stage = Turn;
				}
				// otherwise, find the next bin 
				else
				{
					State::setProperty(State::desiredPower, 100);
					State::setProperty(State::desiredHeading, State::getProperty(State::currentHeading));
					stage = FindNextBin;
					currentBin = 1;
				}
			}
			return;
		}
		case Turn:
		{
			State::setProperty(State::desiredHeading, State::getEquivalentAngle(initialHeading + endAngleOffset));
			pause = 5000;
			stage = Proceed;
			return;
		}
		case Proceed:
		{
			State::setProperty(State::desiredPower, 200);
			pause = endPauseTime;
			stage = Finish;
			return;
		}
		case Finish:
		{
			isCompleted = true;
			return;
		}
	}
}

std::vector<Blob*> Dropper::getBlobs()
{
	cv::Mat img_gray, img_bw(pic_height, pic_width, CV_8UC1), img_in(pic_height, pic_width, CV_8UC1);
	// grab grayscale image
	cvtColor(State::getImage(State::imageDown), img_gray, CV_RGB2GRAY);
	// set to white if it is not a bin border
	pool.add_task(&Dropper::thresholdColor, this,              0,   image_size/4, img_gray.ptr(), img_bw.ptr(), minThresBorder, 255, 0);
	pool.add_task(&Dropper::thresholdColor, this,   image_size/4,   image_size/2, img_gray.ptr(), img_bw.ptr(), minThresBorder, 255, 0);
	pool.add_task(&Dropper::thresholdColor, this,   image_size/2, 3*image_size/4, img_gray.ptr(), img_bw.ptr(), minThresBorder, 255, 0);
	pool.add_task(&Dropper::thresholdColor, this, 3*image_size/4,     image_size, img_gray.ptr(), img_bw.ptr(), minThresBorder, 255, 0);
	pool.wait();

	// filter borders and find centroid
	int cx = 0;
	int cy = 0;
	std::vector<Blob*> borders;
	for (Blob* b : blob_detection(img_bw, 0, 1))
	{
		// check area
		if (b->area < minBorderArea || b->area > maxBorderArea)
		{
			delete b;
			continue;
		}

		// if it passes checks, count it as a border
		cx += b->getCentroid().x;
		cy += b->getCentroid().y;
		borders.push_back(b);
	}
	if (borders.size() > 0)
	{
		cx /= borders.size();
		cy /= borders.size();
	}

	// invert image
	pool.add_task(&Dropper::thresholdColor, this,              0,   image_size/4, img_bw.ptr(), img_in.ptr(), 128, 255, 1);
	pool.add_task(&Dropper::thresholdColor, this,   image_size/4,   image_size/2, img_bw.ptr(), img_in.ptr(), 128, 255, 1);
	pool.add_task(&Dropper::thresholdColor, this,   image_size/2, 3*image_size/4, img_bw.ptr(), img_in.ptr(), 128, 255, 1);
	pool.add_task(&Dropper::thresholdColor, this, 3*image_size/4,     image_size, img_bw.ptr(), img_in.ptr(), 128, 255, 1);
	pool.wait();

	processedImage = img_in;

	// filter bins
	std::vector<Blob*> bins;
	for (Blob* b : blob_detection(img_in, 0, 1))
	{
		// check area
		if (b->area < minBinArea || b->area > maxBinArea)
		{
			delete b;
			continue;
		}

		// check if it is dense
		if ((float)b->area / b->getConvexHullArea() < density)
		{
lprintf("% % %\n", b->area, b->getConvexHullArea(), density);
IM_HERE
			delete b;
			continue;
		}

		// check if it is inside a border
		bool inside = false;
		for (Blob* o : borders)
		{
			if (b->getCentroid().x > o->min_x && b->getCentroid().x < o->max_x && b->getCentroid().y > o->min_y && b->getCentroid().y < o->max_y)
			{
				inside = true;
			}
		}
		if (!inside)
		{
IM_HERE
			delete b;
			continue;
		}

		// if it passes all checks, it is probably a bin
		bins.push_back(b);
		b->drawBlob(processedImage, cv::Scalar(255));
	}

	lprintf("getBlobs: found % bins\n", bins.size());
	return bins;
}

/*
 * Centers the submarine over all the bins 
 */
bool Dropper::goToBins()
{
	std::vector<Blob*> blobs = getBlobs();

	// if there aren't any bins, try again later
	if(blobs.size() == 0)
	{
		lprintf("goToBins: Don't see any blobs yet. moving forward\n");
		State::setProperty(State::desiredPower, searchPower);
		State::setProperty(State::desiredHeading, State::getProperty(State::currentHeading));
		return false;
	}

	int xCenter = 0;
	int yCenter = 0;

	// find average centroid
	for(unsigned int i = 0; i < blobs.size(); i++)
	{
		xCenter += blobs[i]->getCentroid(0).x;
		yCenter += blobs[i]->getCentroid(0).y;
		// and draw the blob on processedImage
		blobs[i]->drawBlob(processedImage, cv::Scalar(255,255,255));
	}

	xCenter /= blobs.size();
	yCenter /= blobs.size();

	// we're done with blobs, so delete
	std::remove_if(blobs.begin(), blobs.end(), [](Blob* b){delete b; return true;});
	
	// write centroid location on processedImage
	std::string coordinates = "Centroid: (" + std::to_string(xCenter) + ", " + std::to_string(yCenter) + ")";
	cv::putText(processedImage, coordinates, cvPoint(20, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,200,200), 1, CV_AA);

	int distanceFromCenterX = xCenter - (pic_width/2);
	int distanceFromCenterY = (pic_height/2) - yCenter;
	float distanceFromCenter = sqrt(distanceFromCenterX * distanceFromCenterX + distanceFromCenterY * distanceFromCenterY) / pic_diag;
	int angle = atan((float)distanceFromCenterY/distanceFromCenterX) * 180 / CV_PI - 90;
	
	// if close enough, continue
	if (distanceFromCenter < distAllBins)
	{
		lprintf("goToBins: centered with angle offset = % and distance = %\n", angle, distanceFromCenter);
		State::setProperty(State::desiredPower, 100);
		State::setProperty(State::desiredHeading, State::getProperty(State::currentHeading));
		return true;
	}

	if (angle > 90)
	{
		angle -= 180;
	}
	if (angle < -90)
	{
		angle += 180;
	}

	if (abs(angle) < aligned)
	{
		// set speed proportional to vertical distance
		State::setProperty(State::desiredPower, (100 + approachPower * distanceFromCenterY / pic_height));
		lprintf("goToBins: distance = %\n", distanceFromCenter);
		return false;
	}
	else
	{
		State::setProperty(State::desiredPower, 100 + slowApproachMultiplier * approachPower * distanceFromCenterY / pic_height);
		// not aligned, so point towards the centroid
		State::setProperty(State::desiredHeading, State::getEquivalentAngle(State::getProperty(State::currentHeading) - angle));
		lprintf("goToBins: angle offset = %\n", angle);
		return false;
	}
}

float getAngle(float angle)
{
	while (angle > 90)
	{
		angle -= 180;
	}
	while (angle <= -90)
	{
		angle += 180;
	}
	return angle;
}

/*
 * Finds the angle of the longer side of rectangle
 * Requires list of lines detected with Hough
 * Returns with format: + is right, 0 is vertical
 */
int Dropper::findLongAngle(const std::vector<cv::Vec2f>& lines)
{
	std::vector<float> angles;
	std::vector<int> weight;
	std::vector<float> magnitudes;

	// group angles
	for (unsigned int i = 0; i < lines.size(); i++)
	{
		float theta = getAngle(lines[i][1] * 180 / CV_PI + 90);
	
		bool flag = false;
		for (unsigned int j = 0; j < angles.size(); j++)
		{
			if (abs(getAngle(theta - angles[j])) < 30)
			{ // they have the same angle
				weight[j]++;
				angles[j] += (theta - angles[j]) / weight[j];
				magnitudes[j] += (lines[i][0] - magnitudes[j]) / (float)weight[j];
				flag = true;
			}
		}
		if (!flag)
		{ // it didn't match any angles, so add it to the list
			angles.push_back(theta);
			magnitudes.push_back(lines[i][0]);
			weight.push_back(1);
		}
	}

	// find longest angle
	int desiredAngle = -1;
	float max = 0;

	for (unsigned int i = 0; i < weight.size(); i++)
	{
		if (weight[i] > max)
		{
			max = weight[i];
			desiredAngle = i;
		}
	}
	
	if (desiredAngle == -1 || desiredAngle >= angles.size())
	{
		lprintf("findLongAngle: something went wrong here... (%)\n", desiredAngle);
		return 0;
	}

	int degrees = angles[desiredAngle];
	
	lprintf("findLongAngle: found % distinct angles, chose %\n", angles.size(), degrees);

	return State::getEquivalentAngle(degrees);
	
}

/*
 * Moves towards the next bin 
 */
bool Dropper::findNextBin()
{
	forceSaveFrame = true;

	// go clockwise

	std::vector<Blob*> blobs = getBlobs();

	if (blobs.size() == 0)
	{
		lprintf("findNextBin: can't find the bins...\n");
		State::setProperty(State::desiredPower, 100);
		State::setProperty(State::desiredHeading, State::getProperty(State::currentHeading));
		return false;
	}

	// find centroid
	int totalArea = 0;
	float centroidX = 0;
	float centroidY = 0;
	for (unsigned int i = 0; i < blobs.size(); i++)
	{
		int thisArea = blobs[i]->area;
		if (thisArea > minBinArea)
		{
			totalArea += thisArea;
			centroidX += (blobs[i]->getCentroid().x - centroidX) * thisArea / totalArea;
			centroidY += (blobs[i]->getCentroid().y - centroidY) * thisArea / totalArea;
			blobs[i]->drawBlob(processedImage, cv::Scalar(255, 255, 255));
		}
		else
		{
			break;
		}
	}

	blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());

	centroidX = centroidX - pic_width / 2;
	centroidY = pic_height / 2 - centroidY + cameraOffset * pic_height;
	
	int angleOfCentroid = 90 - atan2(centroidY, centroidX) * 180 / CV_PI;
	State::setProperty(State::desiredHeading, State::getEquivalentAngle(State::getProperty(State::currentHeading) + angleOfCentroid + goToBinAngle));
	pause = adjustHeadingTime;
	return true;
}

/* 
 * Sets power until the submarine is over the desired bin,
 * aligns to the bins, then lowers in depth a bit.
 */
bool Dropper::centerOverBin()
{
	std::vector<Blob*> blobs = getBlobs();
	
	// if there are no blobs, then try again, because there should be some
	if (blobs.size() == 0)
	{
		// we're done with blobs, so delete
		std::remove_if(blobs.begin(), blobs.end(), [](const Blob* b){delete b; return true;});
		lprintf("centerOverBin: didn't find any blobs\n");
		return false;
	}

	// find blob that is closest to the middle of the image
	int desiredBlob = -1;
	float minDist = 10000000;
	for(int i = 0; i < blobs.size(); i++)
	{
		int dist = pow(blobs[i]->getCentroid(0).x - pic_width / 2, 2) + pow(blobs[i]->getCentroid(0).y - cameraOffset * pic_height - pic_height / 2, 2);
		if (dist < minDist)
		{
			minDist = dist;
			desiredBlob = i;
		}
	}
	minDist = sqrt(minDist) / pic_diag;

	if (desiredBlob == -1)
	{
		// we didn't find any close blobs
		lprintf("centerOverBin: didn't find any reasonably close bins\n");
		return false;
	}
	int bx = blobs[desiredBlob]->getCentroid(0).x;
	int by = blobs[desiredBlob]->getCentroid(0).y - cameraOffset * pic_height;
	int dx = bx - pic_width / 2;
	int dy = pic_height / 2 - by; // cuz cv uses top left as origin, not bot left
	int angleOffset = 90 - atan((float)dy/dx) * 180 / CV_PI;

	// draw it
	blobs[desiredBlob]->drawBlob(processedImage, cv::Scalar(255,255,255));

	// we're done with blobs, so delete
	std::remove_if(blobs.begin(), blobs.end(), [](const Blob* b){delete b; return true;});

	// if close enough, move on
	// otherwise, go towards the bin at a speed proportional to distance
	if (minDist < distBin)
	{
		lprintf("centerOverBin: % is close enough, moving on\n", minDist);
		State::setProperty(State::desiredPower, State::getProperty(State::currentPower));
		State::setProperty(State::desiredHeading, State::getProperty(State::currentHeading));
		return true;
	}
	else
	{
		if (abs(angleOffset) > aligned && minDist > 2 * distBin || abs(angleOffset) > 60)
		{
			State::setProperty(State::desiredPower, 100 + slowApproachMultiplier * approachPower * dy / pic_height);
			State::setProperty(State::desiredHeading, State::getEquivalentAngle(State::getProperty(State::currentHeading) + angleOffset));
			lprintf("centerOverBin: Angle Offset = %\n", angleOffset);
		}
		else
		{
			State::setProperty(State::desiredHeading, State::getProperty(State::currentHeading));
			State::setProperty(State::desiredPower, 100 + approachPower * dy / pic_height);
			lprintf("centerOverBin: Current Distance from Centroid: %\n", minDist);
		}
		return false;
	}
}

/*
 * Returns the id of the image that is detected 
 */
Dropper::DetectedImage Dropper::detectImage()
{
	if (!useImageDetection)
	{
		return DetectedImage::Drop;
	}
	forceSaveFrame = true;
	// this should separate the bin background and everything else (including the image)
	cv::Mat img_gray, img_bw(pic_height, pic_width, CV_8UC1);
	// grab grayscale image
	cvtColor(State::getImage(State::imageDown), img_gray, CV_RGB2GRAY);
	// if under max, 255, else 0
	cv::threshold(img_gray, img_bw, maxThresBin, 255, 1);

	std::vector<Blob*> blobs1 = blob_detection(img_bw, 0, 1);

	// remove small blobs
	for (unsigned int i = 0; i < blobs1.size(); i ++)
	{
		if (blobs1[i]->area < minBinArea)
		{
			blobs1.erase(std::remove_if(blobs1.begin() + i, blobs1.end(), [](const Blob* b){delete b; return true;}), blobs1.end());
			break;
		}
	}

	// find blob that is closest to the middle of the image
	int desiredBlob = -1;
	int minDist = 10000000;
	for(int i = 0; i < blobs1.size(); i++)
	{
		int dist = pow(blobs1[i]->getCentroid(0).x - pic_width / 2, 2) + pow(blobs1[i]->getCentroid(0).y - pic_height / 2, 2);
		if (dist < minDist)
		{
			minDist = dist;
			desiredBlob = i;
		}
	}

	if (desiredBlob == -1)
	{
		// we didn't find any close blobs
		lprintf("detectImage: didn't find any reasonably close bins\n");
		return DetectedImage::Inaccurate;
	}
	
	// initial crop to speed up edge detection
	int minx = blobs1[desiredBlob]->min_x;
	int miny = blobs1[desiredBlob]->min_y;
	int maxx = blobs1[desiredBlob]->max_x;
	int maxy = blobs1[desiredBlob]->max_y;
	// we're done with blobs, so delete
	std::remove_if(blobs1.begin(), blobs1.end(), [](const Blob* b){delete b; return true;});
	int width = maxx - minx;
	int height = maxy - miny;
	cv::Rect canvas1(minx, miny, width, height);
	img_bw = img_bw(canvas1);

	// find the edges
	cv::Mat edge;
	cv::Canny(img_bw, edge, canthres, canratio*canthres, 3, false);

	std::vector<cv::Vec2f> lines;
	// find lines with the edges and find their angle
	cv::HoughLines(edge, lines, 1, CV_PI/180, linethres * edge.rows / img_gray.rows);
	int angle = findLongAngle(lines);
	// rotate to rectangle angle
	lprintf("detectImage: rotating by % degrees\n", angle);
	cv::Point2f src_center(edge.cols/2.0F, edge.rows/2.0F);
	cv::Mat rot;
	cv::warpAffine(img_bw, rot, getRotationMatrix2D(src_center, angle + 90, 1.0), edge.size());

	// crop it again to remove exterior
	std::vector<Blob*> blobs2 = blob_detection(rot, 0, 1);
	if (blobs2.size() == 0)
	{
		lprintf("detectImage: No blobs found on second crop\n");
		return DetectedImage::Inaccurate;
	}
	minx = blobs2[0]->min_x;
	miny = blobs2[0]->min_y;
	width = blobs2[0]->max_x - minx;
	height = blobs2[0]->max_y - miny;
	// we're done with blobs, so delete
	std::remove_if(blobs2.begin(), blobs2.end(), [](const Blob* b){delete b; return true;});
	cv::Rect canvas2(minx, miny, width, height);
	cv::Mat rec = rot(canvas2);

	// save the cropped image
	cv::resize(rec, processedImage, processedImage.size());
	
	float ratio = 0;
	int match = 0;
	int orientation = -1;
	for (int j = 0; j<4; j++)
	{
		// load sample images for comparison
		cv::Mat sample;
		switch (j)
		{
		case 0:
			sample = cv::imread(imgFile[0], CV_LOAD_IMAGE_COLOR);
			break;
		case 1:
			sample = cv::imread(imgFile[1], CV_LOAD_IMAGE_COLOR);
			break;
		case 2:
			sample = cv::imread(imgFile[2], CV_LOAD_IMAGE_COLOR);
			break;
		case 3:
			sample = cv::imread(imgFile[3], CV_LOAD_IMAGE_COLOR);
			break;
		}
		cv::cvtColor(sample, sample, CV_RGB2HSV);  
		cv::inRange(sample, cv::Scalar(0, 0, 0), cv::Scalar(1, 255, 255), sample);

		// we want to go through all 4 orientations
		// because we don't know what rotation it is
		for (int i= 0; i<4; i++)
		{
			// rotate the sample 90 degrees
			cv::Mat smp;
			cv::transpose(sample, smp);
			cv::flip(smp, smp, 1);
			sample = smp;

			// resize it so it can be compared
			cv::Mat resized;
			cv::resize(rec, resized, smp.size());

			// compare
			cv::Mat result;
			int pixels = (double)(smp.rows * smp.cols);
			cv::compare(resized, smp, result, cv::CMP_EQ);
			int similarPixels  = countNonZero(result);
			// if it is closer than previously,
			// set it to match
			if ((double)similarPixels/pixels>ratio)
			{
				ratio = (double)similarPixels/pixels;
				match = j;
				orientation = i;
			}
		}
	}
	lprintf("detectImage: closest match is % rotated % with accuracy %\n", match, orientation * 90, ratio);
	// if the match isnt close enough, its inaccurate
	if (ratio < accuracyThreshold)
	{
		lprintf("detectImage: cannot accurately detect image\n");
		return DetectedImage::Inaccurate;
	}
	// if it is a valid bin OR we are running out, drop
	else if (isValid[match] || (currentBin - State::getProperty(State::dropperState) >= 2))
	{
		lprintf("detectImage: dropping\n");
		return DetectedImage::Drop;
	}
	// otherwise, dont drop
	else
	{
		lprintf("detectImage: please don't drop\n");
		return DetectedImage::DontDrop;
	}
}

void Dropper::thresholdColor(int start, int end, unsigned char* imgIn, unsigned char* imgOut, int thresholdValue, int maxVal, int type)
{
	for(int i = start; i < end; i++)
	{
		switch(type)
		{
			case 0:
				imgOut[i] = (thresholdValue < imgIn[i]) ? maxVal : 0;
				break;
			case 1:
				imgOut[i] = (thresholdValue < imgIn[i]) ? 0 : maxVal;
				break;
			case 2:
				imgOut[i] = (thresholdValue < imgIn[i]) ? thresholdValue : imgIn[i];
				break;
			case 3:
				imgOut[i] = (thresholdValue < imgIn[i]) ? imgIn[i] : 0;
				break;
			case 4:
				imgOut[i] = (thresholdValue < imgIn[i]) ? 0 : imgIn[i];
				break;
			default:
				break;
		}
	}
}
