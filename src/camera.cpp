/*
                                                      
  ___ __ _ _ __ ___   ___ _ __ __ _   ___ _ __  _ __  
 / __/ _` | '_ ` _ \ / _ \ '__/ _` | / __| '_ \| '_ \ 
| (_| (_| | | | | | |  __/ | | (_| || (__| |_) | |_) |
 \___\__,_|_| |_| |_|\___|_|  \__,_(_)___| .__/| .__/ 
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
#include "camera.hpp"
#include "timer.hpp"

#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <cerrno>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <assert.h>

#include <linux/videodev2.h>

#define CLEAR(x) memset(&(x), 0, sizeof (x))

Camera::Camera(const char* name, int w, int h) :
	isCapturing(true),
	buffers(nullptr),
	fd(-1),
	io(IO_MMAP),
	numBuffers(0),
	timpl(new ThreadImpl),
	width(w),
	height(h),
	index(-1)
{
	strcpy(devName, name);
	lprintf("Started vision on %\n", devName);

	// not sure how big latestBuffer needs to be, making it bigger for now just in case
	latestBuffer = malloc(height*width*3);
	dropFrames = 10;
	openDevice();
	initDevice();
	startCapturing();
	timpl->m_thread = std::thread(std::bind(&Camera::runVision, this));
}

Camera::~Camera()
{
	timpl->stop_requested = true;
	timpl->m_thread.join();

	if (isCapturing)
	{
		stopCapturing();
		uninitDevice();
		closeDevice();
	}
	
	lprintf("Camera has closed on %\n", devName);
}

void Camera::pauseCapturing()
{
	isCapturing = false;
	stopCapturing();
	uninitDevice();
	closeDevice();
	
	lprintf("Camera has paused on %\n", devName);
}
void Camera::resumeCapturing()
{
	// drop some frames for auto exposure
	dropFrames = 5;
	openDevice();
	initDevice();
	startCapturing();
	isCapturing = true;
	while (dropFrames) 
	{
		Timer::sleep(5);
	};
	lprintf("Camera has resumed on %\n", devName);
}


void Camera::openDevice()
{
	struct stat st; 
	
	if (-1 == stat (devName, &st))
	{
		fprintf (stderr, "Cannot identify '%s': %d, %s\n",
					devName, errno, strerror (errno));
		exit (EXIT_FAILURE);
	}
	if (!S_ISCHR (st.st_mode))
	{
		fprintf (stderr, "%s is no device\n", devName);
		exit (EXIT_FAILURE);
	}
	
	fd = open (devName, O_RDWR // required 
			| O_NONBLOCK, 0);
	
	struct v4l2_input input;
	
	memset (&input, 0, sizeof (input));
	if (-1 == ioctl (fd, VIDIOC_G_INPUT, &input.index))
	{
		perror ("VIDIOC_G_INPUT");
		exit (EXIT_FAILURE);
	}
	if (-1 == ioctl (fd, VIDIOC_ENUMINPUT, &input))
	{
		perror ("VIDIOC_ENUM_INPUT");
		exit (EXIT_FAILURE);
	}
	if (-1 == fd)
	{
		fprintf (stderr, "Cannot open '%s': %d, %s\n",
					devName, errno, strerror (errno));
		exit (EXIT_FAILURE);
	}
}

void Camera::initDevice()
{
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;

	unsigned int min;
	if (-1 == xioctl (VIDIOC_QUERYCAP, &cap))
	{
		if (EINVAL == errno)
		{
			fprintf (stderr, "%s is no V4L2 device\n", devName);
			exit (EXIT_FAILURE);
		}
		else
		{
			errno_exit ("VIDIOC_QUERYCAP");
		}
	}
	
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
	{
		fprintf (stderr, "%s is no video capture device\n", devName);
		exit (EXIT_FAILURE);
	}
	
	switch (io)
	{
		case IO_READ:
			if (!(cap.capabilities & V4L2_CAP_READWRITE))
			{
				fprintf (stderr, "%s does not support read i/o\n", devName);
				exit (EXIT_FAILURE);
			}
			break;
		
		case IO_MMAP:
		case IO_USERPTR:
			if (!(cap.capabilities & V4L2_CAP_STREAMING))
			{
				fprintf (stderr, "%s does not support streaming i/o\n", devName);
				exit (EXIT_FAILURE);
			}
			break;
	}
	
	
	// select video input, video standard and tune here 
	
	CLEAR (cropcap);
	
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
	if (0 == xioctl (VIDIOC_CROPCAP, &cropcap))
	{
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; // reset to default 
	
		if (-1 == xioctl (VIDIOC_S_CROP, &crop))
		{
			switch (errno)
			{
				case EINVAL:
				// cropping not supported 
				break;
				default:
				// errors ignored 
				break;
			}
		}
	}
	
	struct v4l2_format fmt;
	
	
	CLEAR (fmt);
	
	fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = width; 
	fmt.fmt.pix.height      = height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
	
	if (-1 == xioctl (VIDIOC_S_FMT, &fmt)) errno_exit ("VIDIOC_S_FMT");
	
	// note VIDIOC_S_FMT may change width and height
	
	// buggy driver paranoia.
	min = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;
	
	// autoexposure from Jeff's code
	struct v4l2_queryctrl queryctrl;
	struct v4l2_control control;

	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = V4L2_CID_EXPOSURE_AUTO;
	
	if (-1 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl))
	{
		if (errno != EINVAL)
		{
			perror ("VIDIOC_QUERYCTRL");
			exit (EXIT_FAILURE);
		}
		else
		{
			printf ("V4L2_CID_EXPOSURE_AUTO is not supported\n");
		}
	}
	else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
	{
		printf ("V4L2_CID_EXPOSURE_AUTO is not supported\n");
	}
	else
	{
		memset (&control, 0, sizeof (control));
		control.id = V4L2_CID_EXPOSURE_AUTO;
		control.value = 1;
		control.value = 3;
	
		if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) 
		{
			perror ("VIDIOC_S_CTRL");
			exit (EXIT_FAILURE);
		}
	}
	
	switch (io)
	{
        case IO_MMAP:
                initMMap();
                break;
		default:
			lprintf("SEE ORIGINAL CODE FOR OTHER MEMORY METHODS");
			exit(1);
	}
}

void Camera::initMMap()
{
	struct v4l2_requestbuffers req;
	
	CLEAR (req);
	
	req.count               = 4;
	req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory              = V4L2_MEMORY_MMAP;
	
	if (-1 == xioctl (VIDIOC_REQBUFS, &req))
	{
		if (EINVAL == errno)
		{
			fprintf (stderr, "%s does not support memory mapping\n", devName);
			exit (EXIT_FAILURE);
		}
		else
		{
			errno_exit ("VIDIOC_REQBUFS");
		}
	}
	
	if (req.count < 2)
	{
		fprintf (stderr, "Insufficient buffer memory on %s\n", devName);
		exit (EXIT_FAILURE);
	}
	
	buffers = (buffer *)calloc (req.count, sizeof (*buffers));
	
	if (!buffers)
	{
		fprintf (stderr, "Out of memory\n");
		exit (EXIT_FAILURE);
	}
	
	for (numBuffers = 0; numBuffers < req.count; ++numBuffers)
	{
		struct v4l2_buffer buf;
	
		CLEAR (buf);
	
		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = numBuffers;
	
		if (-1 == xioctl (VIDIOC_QUERYBUF, &buf))
			errno_exit ("VIDIOC_QUERYBUF");
	
		buffers[numBuffers].length = buf.length;
		buffers[numBuffers].start = 
			mmap (	nullptr, // start anywhere
					buf.length,
					PROT_READ | PROT_WRITE, // required 
					MAP_SHARED, // recommended
					fd, buf.m.offset);
	
		if (MAP_FAILED == buffers[numBuffers].start)
			errno_exit ("mmap");
	}
}

void Camera::startCapturing()
{
	unsigned int i;
	enum v4l2_buf_type type;
	
	switch (io)
	{
		case IO_MMAP:
			for (i = 0; i < numBuffers; ++i)
			{
				struct v4l2_buffer buf;
				
				CLEAR (buf);
				
				buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				buf.memory      = V4L2_MEMORY_MMAP;
				buf.index       = i;
				
				if (-1 == xioctl (VIDIOC_QBUF, &buf))
					errno_exit ("VIDIOC_QBUF");
			}
			
			type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			
			if (-1 == xioctl (VIDIOC_STREAMON, &type))
				errno_exit ("VIDIOC_STREAMON");
			
			break;

		default:
			lprintf("SEE ORIGINAL CODE FOR OTHER MEMORY METHODS");
			exit(1);
	}
}

void* Camera::getImageBuffer()
{
	return latestBuffer;
}

int Camera::readFrame()
{
	struct v4l2_buffer buf;
	
	if (io == IO_MMAP)
	{
		CLEAR (buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		
		if (-1 == xioctl (VIDIOC_DQBUF, &buf))
		{
			switch (errno)
			{
				case EAGAIN:
					return 0;
				case EIO:
					// could ignore EIO, see spec
					// fall through
				default:
					errno_exit ("VIDIOC_DQBUF");
			}
		}
		assert (buf.index < numBuffers);
		if (dropFrames > 0)
		{
			dropFrames--;
		}
		else
		{
			index = buf.index;
		}
		if (-1 == xioctl (VIDIOC_QBUF, &buf))
			errno_exit ("VIDIOC_QBUF");
	}
	else
	{
		lprintf("SEE ORIGINAL CODE FOR OTHER MEMORY METHODS");
		exit(1);
	}	
	return 1;
}

int Camera::getIndex()
{
	return index;
}


void Camera::runVision()
{
	while (!timpl->stop_requested)
	{
		while (true)
		{
			if (!isCapturing)
			{
				break;
			}
			struct timeval tv;
			fd_set fds;
			tv.tv_sec = 2;
			tv.tv_usec = 0;

			FD_ZERO(&fds);
			FD_SET(fd, &fds);

			int r = select(fd + 1, &fds, nullptr, nullptr, &tv);

			// if select failed
			if (r == -1)
			{
				if (errno == EINTR) continue;
				errno_exit ("select");
			}

			// if select times out (2 sec)
			if (r == 0)
			{
				fprintf(stderr, "select timeout\n");
				exit (EXIT_FAILURE);
			}
			if (isCapturing && readFrame()) break;
		}
	}
}



void Camera::stopCapturing()
{
	enum v4l2_buf_type type;

	switch (io)
	{
		case IO_READ:
			// nothing to do.
			break;
	
		case IO_MMAP:
		case IO_USERPTR:
			type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (-1 == xioctl (VIDIOC_STREAMOFF, &type))
				errno_exit ("VIDIOC_STREAMOFF");
			break;
	}
}

void Camera::uninitDevice()
{
	unsigned int i;
	
	switch (io)
	{
		case IO_READ:
			free (buffers[0].start);
			break;		
		case IO_MMAP:
			for (i = 0; i < numBuffers; ++i)
				if (-1 == munmap (buffers[i].start, buffers[i].length))
					errno_exit ("munmap");
			break;
			
		case IO_USERPTR:
			for (i = 0; i < numBuffers; ++i)
				free (buffers[i].start);
			break;
	}
	
	free (buffers);
}

void Camera::closeDevice()
{
	if (-1 == close (fd))
		errno_exit ("close");
	fd = -1;
}


void Camera::errno_exit (const char *s)
{
	fprintf (stderr, "%s error %d, %s\n", s, errno, strerror (errno));
	exit (EXIT_FAILURE);
}


/*
 * used to either fill or empty the buffer
 */
int Camera::xioctl(int request, void *arg)
{
	int r;
	do r = ioctl (fd, request, arg);
	while (-1 == r && EINTR == errno);
	return r;
}
