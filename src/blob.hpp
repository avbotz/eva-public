/*
 _     _       _      _                 
| |__ | | ___ | |__  | |__  _ __  _ __  
| '_ \| |/ _ \| '_ \ | '_ \| '_ \| '_ \ 
| |_) | | (_) | |_) || | | | |_) | |_) |
|_.__/|_|\___/|_.__(_)_| |_| .__/| .__/ 
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
#ifndef BLOB_H__
#define BLOB_H__

#include <set>

#include <cv.h>

#include "types.hpp"
	

struct Blob
{
public:
	Blob(int color, int channels, std::vector<cv::Point> blobPoints, std::vector<cv::Point> perimeter, cv::Mat src);
	// combine two blobs
	Blob(Blob* a, Blob* b, int channels);
	// create from CBlob
	Blob(int color, int channels);
	Blob(const Blob &b);
	
	~Blob();
	int min_x, max_x, min_y, max_y;
	unsigned int area, perimeter;
	float bounding_box_fill;
	int num_channels;
	
	long* moment,* moment_x,* moment_y;	// first moments with respect to x and y axes (y = 0  and x = 0)
	
	int getColor();
	cv::Point getCentroid(int color);
	cv::Point getCentroid();
	float getDensity(int channel);
	float getDensity();
	
	bool overlapsWith(Blob* b);
	void addToThis(Blob* b);
	// does not scale internal points; only use after combing blobs
	void scale(int n);
	
	void drawBlob(cv::Mat img, cv::Scalar color);
	void drawContour(cv::Mat img, cv::Scalar color);
	void drawRotatedRect(cv::Mat img, cv::Scalar color);
	
	std::vector<cv::Point> convexHull;
	cv::RotatedRect minSurroundingRect();
	
	float getConvexHullPerimeter();
	float getConvexHullArea();
	float getHullCircularity();
private:

	void calculateConvexHull(std::vector<cv::Point> points);
	
	std::vector<cv::Vec<float, 2> > rotatePoints(std::vector<cv::Vec<float, 2> > points, cv::Vec<float, 2> unitVec, cv::Vec<float, 2> orthoVec);
	int findNextSide(std::vector<cv::Vec<float, 2> >points, int sidesIndeces[4]);
	float dot(cv::Vec<float, 2> a, cv::Vec<float, 2> b);
	cv::Vec<float, 2> unitVector(cv::Vec<float, 2> v);	
	float magnitude(float x, float y);
};

#endif
