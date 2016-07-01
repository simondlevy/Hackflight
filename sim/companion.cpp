/*
   companion.cpp : Companion-board class implementation

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "companion.hpp"

#ifdef __linux

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include <iostream>
using namespace std;

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#endif

// threaded function
static void * _update(void * ptr)
{
    // the function must return something - NULL will do
    return NULL;
}

void CompanionBoard::start(void)
{
}

void CompanionBoard::update(char * imageBytes, int imageWidth, int imageHeight)
{
    Mat image = Mat(imageHeight, imageWidth, CV_8UC3, imageBytes);
    flip(image, image, 0);
    namedWindow( "OpenCV", WINDOW_AUTOSIZE );
    imshow( "OpenCV", image );              
    waitKey(1);     
}

void CompanionBoard::halt(void)
{
}


