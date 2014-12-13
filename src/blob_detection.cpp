/*
 _     _       _           _      _            _   _                               
| |__ | | ___ | |__     __| | ___| |_ ___  ___| |_(_) ___  _ __    ___ _ __  _ __  
| '_ \| |/ _ \| '_ \   / _` |/ _ \ __/ _ \/ __| __| |/ _ \| '_ \  / __| '_ \| '_ \ 
| |_) | | (_) | |_) | | (_| |  __/ ||  __/ (__| |_| | (_) | | | || (__| |_) | |_) |
|_.__/|_|\___/|_.__/___\__,_|\___|\__\___|\___|\__|_|\___/|_| |_(_)___| .__/| .__/ 
                  |_____|                                             |_|   |_|    
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
#include "blob_detection.hpp"

#include <stdint.h>

// offsets needed to go clockwise from a point starting at the point to the right
int8_t cw_order[8][2] = {{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};

void trace_contour(cv::Mat img, cv::Mat labelled, cv::Point start, uint16_t label, std::vector<cv::Point>* perimeter, bool external)
{
	cv::Point second(-1,-1);

	// find the second point in the contour
	// if its an external point we start at position 7, otherwise we start at position 3
	int start_pos = external?7:3;
	int last_pos = start_pos;
	do
	{
		cv::Point current(start.x+cw_order[last_pos][0], start.y+cw_order[last_pos][1]);
		if (img.at<uint8_t>(current) != 0)
		{
			// this is part of the contour
			second = current;
			labelled.at<int16_t>(second) = label;
			if (external)
			{
				perimeter->push_back(cv::Point(second.x-1, second.y-1));
			}
			break;
		}
		else
		{
			// this is not part of the contour
			labelled.at<int16_t>(current) = -1;
		}
		// &0x7 is equivalent to %8
		last_pos = (last_pos+1)&0x7;
	}
	while (last_pos != start_pos);

	if (second == cv::Point(-1,-1))
	{
		// then this was an isolated point
		return;
	}

	cv::Point last = start;
	cv::Point current = second;

	do
	{
		last = current;
		// according to the paper, the start of the next search
		// is at d+2 mod 8 where d is the location of the previous
		// contour pixel with respect to the current pixel
		// since there are 8 positions, adding 4 switches the orientation
		// so d+6 mod 8 is the starting location of the search with respect to
		// the ending location of the previous search
		start_pos = (last_pos + 6) & 0x7;
		last_pos = start_pos;
		do
		{
			current = cv::Point(last.x+cw_order[last_pos][0], last.y+cw_order[last_pos][1]);
			if (img.at<uint8_t>(current) != 0)
			{
				// this is part of the contour
				if (external && labelled.at<int16_t>(current) != label)
				{
					perimeter->push_back(cv::Point(current.x-1, current.y-1));
				}
				labelled.at<int16_t>(current) = label;
				break;
			}
			else
			{
				// this is not part of the contour
				labelled.at<int16_t>(current) = -1;
			}
			last_pos = (last_pos+1)&0x7;
		}
		while (last_pos != start_pos);
	}
	while (last != start && current != second);
}


std::vector<Blob*> blob_detection(cv::Mat img, int color, int channels)
{
	// make 2 images that are larger than the original
	// so that there is a buffer of 1 pixel all the way around
	cv::Mat copy(img.rows+2, img.cols+2, CV_8UC1);
	cv::Mat labelled(img.rows+2, img.cols+2, CV_16SC1);
	int16_t highest_label = 1;

	std::vector<std::vector<cv::Point>> perimetersPoints;

	memset(labelled.ptr(), 0, labelled.rows*labelled.cols*2);
	memset(copy.ptr(), 0, copy.rows*copy.cols);
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			copy.at<uint8_t>(i+1,j+1) = img.at<uint8_t>(i,j);
		}
	}
	
	for (int i = 1; i < copy.rows-1; i++)
	{
		for (int j = 1; j < copy.cols-1; j++)
		{
			if (copy.at<uint8_t>(i,j) != 0)
			{
				// this is not a black pixel
				// step 1: if pixel is unlabelled and above it is a black pixel
				if (labelled.at<int16_t>(i,j) == 0 && (i == 0 || copy.at<uint8_t>(i-1,j) == 0))
				{
					// found new external contour
					labelled.at<int16_t>(i,j) = highest_label;
					std::vector<cv::Point> perimeter;
					perimeter.push_back(cv::Point(j-1,i-1));
					trace_contour(copy, labelled, cv::Point(j,i), highest_label, &perimeter, true);
					perimetersPoints.push_back(perimeter);
					highest_label++;
				}
				// step 2: if pixel is above an unmarked black pixel
				if (copy.at<uint8_t>(i+1,j) == 0 && labelled.at<int16_t>(i+1,j) == 0)
				{
					// found new internal contour	
					if (labelled.at<int16_t>(i,j) == 0)
					{
						// this point does not also lie on an external contour, so we give it the label
						// of the preceeding point
						labelled.at<int16_t>(i,j) = labelled.at<int16_t>(i,j-1);
					}
					// label the internal contour
					trace_contour(copy, labelled, cv::Point(j,i), labelled.at<int16_t>(i,j), nullptr, false);
				}
				// step 3: deal with all pixels not dealt with in 1 or 2
				if (labelled.at<int16_t>(i,j) == 0)
				{
					// this pixel is not on an external or internal contour
					// assign it the same label as the pixel directly to its left
					labelled.at<int16_t>(i,j) = labelled.at<int16_t>(i,j-1);
				}
			}
		}
	}
	
	int num_blobs = highest_label - 1;
	// scan the picture and move the labelled points into blobs
	std::vector<cv::Point>* blobsPoints = new std::vector<cv::Point>[num_blobs];
	for (int i = 1; i < copy.rows-1; i++)
	{
		for (int j = 1; j < copy.cols-1; j++)
		{
			if (labelled.at<int16_t>(i,j) > 0)
			{
				blobsPoints[labelled.at<int16_t>(i,j)-1].push_back(cv::Point(j-1,i-1));
			}
		}
	}
	
	std::vector<Blob*> result;
	for (int i = 0; i < num_blobs; i++)
	{
		if (blobsPoints[i].size() > 0)
		{
			result.push_back(new Blob(color, channels, blobsPoints[i], perimetersPoints[i], img));
		}
	}

	// sort the blobs from greatest area to least area
	std::sort(result.begin(), result.end(), 
		[](Blob* const a, Blob* const b)->bool
		{
			return (a->area > b->area);
		}
	);
	return result;
}
