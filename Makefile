#  __  __       _         __ _ _      
# |  \/  | __ _| | _____ / _(_) | ___ 
# | |\/| |/ _` | |/ / _ \ |_| | |/ _ \
# | |  | | (_| |   <  __/  _| | |  __/
# |_|  |_|\__,_|_|\_\___|_| |_|_|\___|
#
# Copyright (c) 2014 AVBotz
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# =============================================================================

VERSION = 2014-12

VERSION_CFLAG = -DVERSION="\"$(VERSION)\""

EXTERNAL = 3rdparty/

OPENCV_PATH = $(EXTERNAL)/opencv-2.4.9/
OPENCV_BUILD = $(OPENCV_PATH)/build/
OPENCV_LIB = $(OPENCV_BUILD)/lib/
OPENCV_EXTERNAL = $(OPENCV_BUILD)/3rdparty/lib/
OPENCV_CFLAG = \
			   -I$(OPENCV_PATH)/modules/core/include/ \
			   -I$(OPENCV_PATH)/modules/highgui/include/ \
			   -I$(OPENCV_PATH)/modules/imgproc/include/ \
			   -I$(OPENCV_PATH)/modules/ml/include/ \
			   -I$(OPENCV_PATH)/modules/video/include/ \
			   -I$(OPENCV_PATH)/modules/features2d/include/ \
			   -I$(OPENCV_PATH)/modules/flann/include/ \
			   -I$(OPENCV_PATH)/modules/calib3d/include/ \
			   -I$(OPENCV_PATH)/modules/objdetect/include/ \
			   -I$(OPENCV_PATH)/modules/legacy/include/ \
			   -I$(OPENCV_PATH)/include/opencv/ \
			   -I$(OPENCV_PATH)/include/opencv2/ \
			   -I$(OPENCV_PATH)/include/
OPENCV_LFLAG = \
			   $(OPENCV_LIB)/libopencv_imgproc.a \
			   $(OPENCV_LIB)/libopencv_highgui.a \
			   $(OPENCV_LIB)/libopencv_core.a \
			   $(OPENCV_EXTERNAL)/liblibjpeg.a \
			   $(OPENCV_EXTERNAL)/liblibpng.a \
			   $(OPENCV_EXTERNAL)/libzlib.a \
			   -lrt

LIBCONFIG_PATH = $(EXTERNAL)/libconfig-1.4.9/
LIBCONFIG_CFLAG = -I$(LIBCONFIG_PATH)/lib/
LIBCONFIG_LFLAG = $(LIBCONFIG_PATH)/lib/.libs/libconfig++.a

THREAD_CFLAG = -pthread
THREAD_LFLAG = -lpthread

EXTERNAL_CFLAG = -I$(EXTERNAL)

CFLAGS = -c -std=c++11
LFLAGS = 

all: builddir eva

builddir:
	mkdir build

dependencies: opencv libconfig
	touch dependencies

opencv: $(OPENCV_PATH)/CMakeLists.txt
	mkdir $(OPENCV_BUILD)
	cd $(OPENCV_BUILD) && \
		cmake .. \
			-DWITH_GSTREAMER:BOOL="0" \
			-DWITH_OPENCLAMDBLAS:BOOL="0" \
			-DBUILD_TESTS:BOOL="0" \
			-DWITH_CUDA:BOOL="0" \
			-DWITH_FFMPEG:BOOL="0" \
			-DANT_EXECUTABLE:FILEPATH="ANT_EXECUTABLE-NOTFOUND" \
			-DENABLE_SSE:BOOL="0" \
			-DWITH_GTK:BOOL="0" \
			-DBUILD_PACKAGE:BOOL="0" \
			-DWITH_JASPER:BOOL="0" \
			-DWITH_OPENEXR:BOOL="0" \
			-DBUILD_opencv_apps:BOOL="0" \
			-DBUILD_opencv_calib3d:BOOL="0" \
			-DBUILD_opencv_contrib:BOOL="0" \
			-DBUILD_opencv_core:BOOL="1" \
			-DBUILD_opencv_features2d:BOOL="0" \
			-DBUILD_opencv_flann:BOOL="1" \
			-DBUILD_opencv_gpu:BOOL="0" \
			-DBUILD_opencv_highgui:BOOL="1" \
			-DBUILD_opencv_imgproc:BOOL="1" \
			-DBUILD_opencv_legacy:BOOL="0" \
			-DBUILD_opencv_ml:BOOL="0" \
			-DBUILD_opencv_nonfree:BOOL="0" \
			-DBUILD_opencv_objdetect:BOOL="0" \
			-DBUILD_opencv_ocl:BOOL="0" \
			-DBUILD_opencv_photo:BOOL="0" \
			-DBUILD_opencv_stitching:BOOL="0" \
			-DBUILD_opencv_superres:BOOL="0" \
			-DBUILD_opencv_ts:BOOL="0" \
			-DBUILD_opencv_video:BOOL="0" \
			-DBUILD_opencv_videostab:BOOL="0" \
			-DBUILD_opencv_world:BOOL="0" \
			-DWITH_GIGEAPI:BOOL="0" \
			-DWITH_JPEG:BOOL="1" \
			-DBUILD_JPEG:BOOL="1" \
			-DWITH_PNG:BOOL="1" \
			-DBUILD_PNG:BOOL="1" \
			-DBUILD_ZLIB:BOOL="1" \
			-DWITH_CUFFT:BOOL="0" \
			-DBUILD_WITH_DEBUG_INFO:BOOL="1" \
			-DWITH_V4L:BOOL="0" \
			-DWITH_OPENCLAMDFFT:BOOL="0" \
			-DBUILD_PERF_TESTS:BOOL="0" \
			-DBUILD_DOCS:BOOL="0" \
			-DENABLE_OMIT_FRAME_POINTER:BOOL="0" \
			-DWITH_TIFF:BOOL="0" \
			-DWITH_1394:BOOL="0" \
			-DENABLE_PRECOMPILED_HEADERS:BOOL="0" \
			-DWITH_EIGEN:BOOL="0" \
			-DWITH_LIBV4L:BOOL="0" \
			-DENABLE_SSE2:BOOL="0" \
			-DBUILD_FAT_JAVA_LIB:BOOL="0" \
			-DENABLE_SSE3:BOOL="0" \
			-DBUILD_SHARED_LIBS:BOOL="0" \
			-DWITH_OPENCL:BOOL="0" \
			-DWITH_PVAPI:BOOL="0" \
			&& \
		make
	touch opencv

libconfig: $(LIBCONFIG_PATH)/configure
	cd $(LIBCONFIG_PATH) && ./configure && make
	touch libconfig

OBJECTS = build/blob.o build/blob_detection.o build/buoy.o build/camera.o build/dropper.o build/eva.o build/gate.o build/hydrophone.o build/l_gate.o build/log.o build/path.o build/search.o build/serial.o build/serial_mbed.o build/state.o build/task.o build/timer.o build/torpedo.o build/version.o build/vision.o build/vision_normal.o build/vision_sim.o build/visiontask.o
eva: dependencies $(OBJECTS)
	g++ $(OBJECTS) -o eva \
		$(THREAD_LFLAG) \
		$(LIBCONFIG_LFLAG) \
		$(OPENCV_LFLAG) \
		$(LFLAGS)

build/blob.o: src/blob.cpp
	g++  src/blob.cpp -o build/blob.o \
		$(OPENCV_CFLAG) \
		$(CFLAGS)

build/blob_detection.o: src/blob_detection.cpp
	g++  src/blob_detection.cpp -o build/blob_detection.o \
		$(OPENCV_CFLAG) \
		$(CFLAGS)

build/buoy.o: src/buoy.cpp
	g++  src/buoy.cpp -o build/buoy.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/camera.o: src/camera.cpp
	g++  src/camera.cpp -o build/camera.o \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/dropper.o: src/dropper.cpp
	g++  src/dropper.cpp -o build/dropper.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/eva.o: src/eva.cpp
	g++  src/eva.cpp -o build/eva.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/gate.o: src/gate.cpp
	g++  src/gate.cpp -o build/gate.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/hydrophone.o: src/hydrophone.cpp
	g++  src/hydrophone.cpp -o build/hydrophone.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/l_gate.o: src/l_gate.cpp
	g++  src/l_gate.cpp -o build/l_gate.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/log.o: src/log.cpp
	g++  src/log.cpp -o build/log.o \
		$(CFLAGS)

build/path.o: src/path.cpp
	g++  src/path.cpp -o build/path.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/search.o: src/search.cpp
	g++  src/search.cpp -o build/search.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/serial.o: src/serial.cpp
	g++  src/serial.cpp -o build/serial.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/serial_mbed.o: src/serial_mbed.cpp
	g++  src/serial_mbed.cpp -o build/serial_mbed.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/state.o: src/state.cpp
	g++  src/state.cpp -o build/state.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/task.o: src/task.cpp
	g++  src/task.cpp -o build/task.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/timer.o: src/timer.cpp
	g++  src/timer.cpp -o build/timer.o \
		$(CFLAGS)

build/torpedo.o: src/torpedo.cpp
	g++  src/torpedo.cpp -o build/torpedo.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/version.o: src/version.cpp
	g++  src/version.cpp -o build/version.o \
		$(VERSION_CFLAG) \
		$(CFLAGS)

build/vision.o: src/vision.cpp
	g++  src/vision.cpp -o build/vision.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/vision_normal.o: src/vision_normal.cpp
	g++  src/vision_normal.cpp -o build/vision_normal.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/vision_sim.o: src/vision_sim.cpp
	g++  src/vision_sim.cpp -o build/vision_sim.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)

build/visiontask.o: src/visiontask.cpp
	g++  src/visiontask.cpp -o build/visiontask.o \
		$(OPENCV_CFLAG) \
		$(LIBCONFIG_CFLAG) \
		$(THREAD_CFLAG) \
		$(EXTERNAL_CFLAG) \
		$(CFLAGS)


.PHONY: clean cleanest

clean:
	-rm -f build/*.o
	-rm -f eva

cleanest:
	-rm -f build/*.o
	-rm -f eva
	-rm -f dependencies
	-cd $(LIBCONFIG_PATH) && make distclean
	-rm -f libconfig
	-cd $(OPENCV_PATH) && rm -Rf build
	-rm -f opencv
