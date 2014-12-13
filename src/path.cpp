/*
             _   _                       
 _ __   __ _| |_| |__    ___ _ __  _ __  
| '_ \ / _` | __| '_ \  / __| '_ \| '_ \ 
| |_) | (_| | |_| | | || (__| |_) | |_) |
| .__/ \__,_|\__|_| |_(_)___| .__/| .__/ 
|_|                         |_|   |_|    
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
#include "path.hpp"

int Path::frame = 0;

Path::Path(libconfig::Config *c) : VisionTask(c, CAM_DOWN),
	initialHeading(State::getProperty(State::currentHeading)),
	canSeePath(false),	
	startedSearchLeg(false),
	isCheck(false),
	grayImg(pic_height, pic_width, CV_8UC1)
{
	conf->lookupValue("path.min_total_intensity", minTotalIntensity);
	conf->lookupValue("path.follow_time", followTime);
	conf->lookupValue("path.timeout", timeout);
	
	conf->lookupValue("path." + TASK_NAME[State::getProperty(State::missionState)] + ".num_paths", num_paths);
	conf->lookupValue("path." + TASK_NAME[State::getProperty(State::missionState)] + ".preferred_path", preferred_path);
	conf->lookupValue("path." + TASK_NAME[State::getProperty(State::missionState)] + ".proceed_time", proceedTime);
	conf->lookupValue("eva.is_competition_side", midOfPoolIsCCW);

	labelStage("Search for path");
	labelStage("Center over path");
	labelStage("Turn path heading");
	labelStage("Waiting");
	labelStage("Moving on");
	labelStage("Finished");
}

Path::~Path()
{
}

void Path::action()
{
	switch (stage)
	{
	case 0:
	{	
		initializePan(State::getProperty(State::currentHeading), 25);
		currentPanHeading = 0;
		State::setProperty(State::desiredHeading, panHeading[currentPanHeading]);
		State::setProperty(State::desiredPower, 130);
		stage = 1;
	}
	case 1:
	{
		// search for the path
		search();
		return;
	}
	case 2:
	{
		// center the sub over the path
		center();
		return;
	}
	case 3:
	{
		// turn to the correct heading
		follow();
		return;
	}
	case 4:
	{
		// wait for a little bit of time to show we are following the path
		pause = followTime;
		stage = 5;
		return;
	}
	case 5:
	{
		// follow the direction of the path, while moving, because we know that's the correct direction
		State::setProperty(State::desiredPower, 170);
		pause = proceedTime;
		stage = 6;
		return;
	}
	case 6:
	{
		isCompleted = true;
		return;
	}
	}
}

void Path::search()
{
	if (!startedSearchLeg && abs(State::getProperty(State::currentHeading) - State::getProperty(State::desiredHeading)) < 15)
	{
		// if were close to the search angle the start the search at that angle
		timer.resetTimer();
		startedSearchLeg = true;
		State::setProperty(State::desiredPower, 130);
	}
	
	if (startedSearchLeg && timer.getCurrentTime() > 3000)
	{
		// heading timed out
		// in this case we don't want the middle pan heading so we're trying to toggle currentPanHeading back and forth
		// between 1 and 2
		goToNextPanHeading();
		State::setProperty(State::desiredPower, 100);
		
		timer.resetTimer();
		startedSearchLeg = false;
	}
	
	canSeePath = processImage();
	if (canSeePath)
	{
		State::setProperty(State::desiredPower, 100);
		stage = 2;
	}
}


bool Path::check()
{
	if (checkTimer.getCurrentTime() == -1)
	{
		checkTimer.resetTimer();
	}
	if (checkTimer.getCurrentTime() > timeout)
	{
		stage = 5;
		State::setProperty(State::desiredHeading, initialHeading);
		proceedTime -= 4000;
		return true;
		
	}
	return processImage();
}

void Path::thresholdColor(int start, int end, unsigned char* ip, unsigned char* processed)
{
	for (int i = start; i < end; i++)
	{
		int b = ip[3*i], g = ip[3*i+1], r = ip[3*i+2];
		
		int val = 2*r - g - 3*b;
		
		val = THRESHOLD(0, val, 255);
		
		processed[i] = val;
	}
}

bool Path::processImage()
{
	cv::Mat image = State::getImage(State::imageDown);

	long long cogX[4], cogY[4], total[4];
	memset(cogX, 0, sizeof(int)*4);
	memset(cogY, 0, sizeof(int)*4);
	memset(total, 0, sizeof(int)*4);
	
	memset(processedImage.ptr(), 0, processedImage.rows*processedImage.cols*3);

	int image_size = image.rows * image.cols;

	pool.add_task(&Path::thresholdColor, this,              0,   image_size/4, image.ptr(), grayImg.ptr());
	pool.add_task(&Path::thresholdColor, this,   image_size/4,   image_size/2, image.ptr(), grayImg.ptr());
	pool.add_task(&Path::thresholdColor, this,   image_size/2, 3*image_size/4, image.ptr(), grayImg.ptr());
	pool.add_task(&Path::thresholdColor, this, 3*image_size/4,     image_size, image.ptr(), grayImg.ptr());

	pool.wait();

	if(num_paths == 2)
	{
		std::vector<Blob*> blobs = blob_detection(grayImg, 0, 1);

		if(!blobs.size())
		{
			freeBlobs(blobs);
			return false;
		}

		if(blobs.size() > 2)
		{
			blobs.erase(std::remove_if(blobs.begin() + 2, blobs.end(), [](const Blob* b){delete b; return true;}), blobs.end());
		}

		if(blobs.size() == 2)
		{
			int leftBlob = (blobs[0]->getCentroid().x < blobs[1]->getCentroid().x) ? 0 : 1;
			int notPreferredBlob = (preferred_path == 1) ? leftBlob : ((leftBlob == 0) ? 1 : 0);
			
			cv::Point *vertices = new cv::Point[blobs[notPreferredBlob]->convexHull.size()];
			
			for(int i = 0; i < blobs[notPreferredBlob]->convexHull.size(); i++)
			{
				vertices[i] = blobs[notPreferredBlob]->convexHull[i];
			}

			cv::fillConvexPoly(grayImg, vertices, blobs[notPreferredBlob]->convexHull.size(), cv::Scalar(0));

			delete[] vertices;
		}

		freeBlobs(blobs);
	}

	auto doLine = [&](int start, int end, unsigned char* ip, long long* momentY, long long* momentX, long long* total_color, int width) {
		*total_color = 0;
		*momentY = 0;
		*momentX = 0;

		for (int i = start; i < end; i++)
		{
			*momentY += ip[i] * (i%width);
			*momentX += ip[i] * (i/width);
			*total_color += ip[i];

			processedImage.ptr()[3*i] = ip[i];
		}
	};

	pool.add_task(doLine,              0,   image_size/4, grayImg.ptr(),   cogX,   cogY,   total, pic_width);
	pool.add_task(doLine,   image_size/4,   image_size/2, grayImg.ptr(), cogX+1, cogY+1, total+1, pic_width);
	pool.add_task(doLine,   image_size/2, 3*image_size/4, grayImg.ptr(), cogX+2, cogY+2, total+2, pic_width);
	pool.add_task(doLine, 3*image_size/4,     image_size, grayImg.ptr(), cogX+3, cogY+3, total+3, pic_width);

	pool.wait();

	long long mY = cogX[0]+cogX[1]+cogX[2]+cogX[3];
	long long mX = cogY[0]+cogY[1]+cogY[2]+cogY[3];
	long long tot_int = total[0]+total[1]+total[2]+total[3];
	cX =  mY / (double)tot_int;
    cY =  mX / (double)tot_int;
	
	char intensity_str[100];
	sprintf(intensity_str, "%lld", tot_int);
	cv::putText(processedImage, intensity_str, cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,200,200), 1, CV_AA);
	return (tot_int > minTotalIntensity);
}

void Path::center()
{
	if (!processImage())
		return;

	cv::Point midred((int)cX, (int)cY);
	cv::circle(processedImage, cv::Point(cX,cY), 5, cv::Scalar(0, 128, 255), CV_FILLED);
	
	char center_str[100];
	sprintf(center_str, "%f, %f",cX,cY);
	cv::putText(processedImage, center_str, cvPoint(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,200,200), 1, CV_AA);
	
	// the midpoint of the image
	int mid_x = pic_width / 2;
	int mid_y = pic_height / 2;
	// adjust because the camera is mounted in the front
	// calculate how far we are in the x and y directions from center of the path
	double ydist = cY-mid_y;
	double xdist = cX-mid_x;
	// is this amount of error acceptable
	bool close_enough_x = (xdist < 60 && xdist > -60);
	bool close_enough_y = (ydist < 33 && ydist > -33);
	
	int newHeading = State::getProperty(State::currentHeading);
	
	// attempt to turn toward the path
	if (xdist > 60) newHeading += 10;
	if (xdist > 150) newHeading += 10;
	
	if (xdist < -60) newHeading -= 10;
	if (xdist < -150) newHeading -= 10;
	
	newHeading = State::getEquivalentAngle(newHeading);
	State::setProperty(State::desiredHeading, newHeading);
	
	if (close_enough_x && close_enough_y)
	{
		// path is centered
		// proceed to getting the correct heading
		State::setProperty(State::desiredPower, 100);
		stage = 3;
		return;
	}
	
	else if (!close_enough_y && ydist > 0)
	{
		// too far above it
		State::setProperty(State::desiredPower, 70);
	}
	else if (!close_enough_y && ydist < 0)
	{
		// too far below it
		State::setProperty(State::desiredPower, 130);
	}
	else
	{
		// y direction is good
		State::setProperty(State::desiredPower, 100);
	}
}

void Path::detectLines(cv::Mat mat,
	std::vector<std::array<float,3>>* adjusted_lines,
	unsigned int* color_quantity)
{
	*color_quantity = 0;
	for (int i = 0; i < mat.rows; i++)
	{
		for (int j = 0; j < mat.cols; j++)
		{
			if (mat.at<unsigned char>(i,j) > 200)
			{
				(*color_quantity)++;
			}
		}
	}

	// canny is an edge detector that uses the magnitude of the gradient of the colors to
	// emphasize where edges are (because where the colors are changing is an edge)
	cv::Canny(mat, mat, 60, 180);
	
	std::vector<cv::Vec2f> raw_lines;
	
	// hough figures out the slopes of the lines
	// it is basically running a lot of test slopes though the important points and calculating the equation of the line
	// it then figures out which lines appear the most and those are the equations of the edges that appear in the picture
	cv::HoughLines(mat, raw_lines, 1, CV_PI/180, 20, 0, 0);
	
	for (int i = 0; i < raw_lines.size(); i++)
	{
		float slope_adj = raw_lines[i][1];
		if (slope_adj > CV_PI/2)
		{
			slope_adj -= CV_PI;
		}
		adjusted_lines->push_back(std::array<float, 3>{{raw_lines[i][0], raw_lines[i][1], slope_adj}});
	}
}

void Path::follow()
{
	if (!processImage())
	{
		return;
	}
	
	std::vector<std::array<float,3>> lines;
	std::vector<std::array<float,3>> quarter_lines[4];
	
	std::vector<_line> endpoints;
	
	cv::Mat topLeft (grayImg, cv::Range(0, grayImg.rows/2), cv::Range(0, grayImg.cols/2));
	cv::Mat topRight(grayImg, cv::Range(0, grayImg.rows/2), cv::Range(grayImg.cols/2, grayImg.cols));
	cv::Mat botLeft (grayImg, cv::Range(grayImg.rows/2, grayImg.rows), cv::Range(0, grayImg.cols/2));
	cv::Mat botRight(grayImg, cv::Range(grayImg.rows/2, grayImg.rows), cv::Range(grayImg.cols/2, grayImg.cols));

	unsigned int edge_quantity[4], total_edge;

	pool.add_task(&Path::detectLines, this,  topLeft,   quarter_lines,   edge_quantity);
	pool.add_task(&Path::detectLines, this, topRight, quarter_lines+1, edge_quantity+1);
	pool.add_task(&Path::detectLines, this,  botLeft, quarter_lines+2, edge_quantity+2);
	pool.add_task(&Path::detectLines, this, botRight, quarter_lines+3, edge_quantity+3);

	pool.wait();
	
	float edge_difference = float(edge_quantity[0] + edge_quantity[1]) / ((edge_quantity[2] + edge_quantity[3]));
	
	if (edge_difference > 5.0f)
	{
		State::setProperty(State::desiredPower, 130);
	}
	else if (edge_difference < 1/5.0f)
	{
		State::setProperty(State::desiredPower, 70);
	}
	
	total_edge = edge_quantity[0] + edge_quantity[1] + edge_quantity[2] + edge_quantity[3];
	unsigned int edge_per_line = 25;
	// you want a maximum of 100 lines, and 25 pixels of intense color in an area will get you one line
	if (total_edge > 100*edge_per_line)
	{
		edge_per_line = total_edge / 100;
	}
	
	for (int i = 0; i < 4; i++)
	{
		if (edge_quantity[i]/edge_per_line < quarter_lines[i].size())
		{
			lines.insert(lines.end(), quarter_lines[i].begin(), quarter_lines[i].begin() + edge_quantity[i]/edge_per_line);
		}
		else
		{
			lines.insert(lines.end(), quarter_lines[i].begin(), quarter_lines[i].end());
		}
	}
	
	if (lines.size() <= 2) {
		// found very few lines for some reason
		return;
	}
	
	for (unsigned int i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0];
		float theta = lines[i][1];
		//the equation of the line is in rho and theta instead of mx+b because
		//vertical lines cannot be represented in mx+b
		//m = y/x = -cos(theta)/sin(theta), b = rho/sin(theta)
		double a = cos(theta), b = sin(theta);
		//get a point on the line
		double x0 = a*rho, y0 = b*rho;
		//find 2 points on the line by going in opposite directions from the above point
		cv::Point pt1, pt2;
		pt1.x = round(x0 + 1000*(-b));
		pt1.y = round(y0 + 1000*(a));
		pt2.x = round(x0 - 1000*(-b));
		pt2.y = round(y0 - 1000*(a));

		double ydiff = pt2.y - pt1.y;
		double xdiff = pt2.x - pt1.x;
		double slope = ydiff/xdiff;
		
		_line templine = _line(pt1, pt2, slope);

		//draw a line onto the image
		cv::line(processedImage, templine.pt1, templine.pt2, cv::Scalar(0,255,0),1);
	}
	
	//when looking at a path, there are 2 sets of parallel lines, 
	//the long ones we want and the short ones we dont
	//the long lines are longer, so the hough transform should detect more of them
	//by only using lines with a slope within 1 standard deviation of the mean,
	//we can get rid of the other lines
	
	float meanTheta[2] = {0.0f, 0.0f}, sumSqrErr[2] = {0.0f, 0.0f}, stdDev,  meanLongTheta = 0.0f;
	int numLongLines = 0;
	int slopeIndex;
	
	//we need to do this twice, once for the unadjusted slope, one for the adjusted slope
	//then we check whose standard deviation will be smaller, which tells us which
	//set of slopes to use
	for (int j = 0; j < 2; j++)
	{
		for (unsigned int i = 0; i < lines.size(); i++)
		{
			meanTheta[j] += lines[i][j+1];
		}
		meanTheta[j] /= lines.size();
		for (unsigned int i = 0; i < lines.size(); i++)
		{
			sumSqrErr[j] += pow(meanTheta[j] - lines[i][j+1], 2);
		}
	}
	
	slopeIndex = (sumSqrErr[0] < sumSqrErr[1]) ? 0 : 1;
	stdDev = sqrt(sumSqrErr[slopeIndex] / (lines.size() - 1));
	
	for (unsigned int i = 0; i < lines.size(); i++)
	{
		if (fabs(meanTheta[slopeIndex] - lines[i][slopeIndex+1]) < stdDev)
		{
			meanLongTheta += lines[i][slopeIndex+1];
			numLongLines++;
		}
	}
	
	meanLongTheta /= numLongLines;
	
	int degrees = 180*meanLongTheta/CV_PI;
	
	if (degrees > 90)
	{
		degrees -= 180;
	}
	
	
	char degrees_str[8], file_str[16];
	sprintf(degrees_str, "%d",degrees);
	cv::putText(processedImage, degrees_str, cvPoint(30,90), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,255), 1, CV_AA);
	sprintf(file_str, "%d.jpg", frame++);
	
	
	//set the desired heading
	int newHeading = State::getEquivalentAngle(State::getProperty(State::currentHeading) + degrees);
	//you cant have to turn more than 90 degrees in either direction to follow the path otherwise
	//you would be following the path in the wrong direction
	if (State::getAngleDifference(newHeading, initialHeading) > 90)
	{
		newHeading = State::getEquivalentAngle(newHeading - 180);
	}
	else if (State::getAngleDifference(newHeading, initialHeading) < -90)
	{
		newHeading = State::getEquivalentAngle(newHeading + 180);
	}
	
	State::setProperty(State::desiredHeading, newHeading);
	//if were close enough, then were done with path
	if (abs(degrees) < 5)
	{	
		State::setProperty(State::desiredPower, 100);
		stage = 4;
	}
	
}
