/*
 _     _       _           _      _            _   _               _                 
| |__ | | ___ | |__     __| | ___| |_ ___  ___| |_(_) ___  _ __   | |__  _ __  _ __  
| '_ \| |/ _ \| '_ \   / _` |/ _ \ __/ _ \/ __| __| |/ _ \| '_ \  | '_ \| '_ \| '_ \ 
| |_) | | (_) | |_) | | (_| |  __/ ||  __/ (__| |_| | (_) | | | |_| | | | |_) | |_) |
|_.__/|_|\___/|_.__/___\__,_|\___|\__\___|\___|\__|_|\___/|_| |_(_)_| |_| .__/| .__/ 
                  |_____|                                               |_|   |_|    
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

//based on "A linear-time component-labeling algorithm using contour tracing technique" by Chang, et al.

#ifndef BLOB_DETECTION_H__
#define BLOB_DETECTION_H__

#include <cv.h>
#include <vector>

#include "blob.hpp"

void trace_contour(cv::Mat img, cv::Mat labelled, cv::Point start, uint16_t label, std::vector<cv::Point>* perimeter, bool external);

// detects gray blobs on black background
std::vector<Blob*> blob_detection(cv::Mat img, int color, int channels);

#endif
