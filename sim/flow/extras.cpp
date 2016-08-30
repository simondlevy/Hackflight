/*
   extras.cpp : Implementation of extra simulator functionality

   Copyright (C) Simon D. Levy, Matt Lubas, and Julio Hidalgo Lopez 2016

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "extras.hpp"
#include "sockets.hpp"

#include "scriptFunctionData.h"
#include "v_repLib.h"


#include <stdio.h>

// MSP message support
static char mspFromServer[200];
static int  mspFromServerLen;
static int  mspFromServerIndex;

// We use OpenCV to compress JPEG for use by Python script
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

#include <signal.h>
#include <unistd.h>
#include <sys/stat.h>

static const int CAMERA_PORT          = 5000;
static const int COMMS_IN_PORT        = 5001;
static const int COMMS_OUT_PORT       = 5002;
static const char * IMAGE_TO_PYTHON   = "image.jpg";
static const char * IMAGE_FROM_PYTHON = "image2.jpg";
static const int MAXMSG               = 1000;

class CompanionBoard {
    
    private:

        int procid;

        SocketClient cameraSyncSocket;
        SocketClient commsInSocket;
        SocketClient commsOutSocket;

        int imgsize;

    public:

        CompanionBoard(void)
        {
            this->procid = 0;
        }

        void start(void)
        {
            // Build command-line arguments for forking the Python server script
            char script[200];
            sprintf(script, "%s/hackflight_companion.py", VREP_DIR);
            char camera_port[10];
            sprintf(camera_port, "%d", CAMERA_PORT);
            char comms_in_port[10];
            sprintf(comms_in_port, "%d", COMMS_IN_PORT);
            char comms_out_port[10];
            sprintf(comms_out_port, "%d", COMMS_OUT_PORT);
            char *argv[7] = { 
                (char *)script, 
                camera_port, 
                comms_in_port, 
                comms_out_port, 
                (char *)IMAGE_TO_PYTHON, 
                (char *)IMAGE_FROM_PYTHON, 
                NULL};

            // Fork the Python server script
            this->procid = fork();
            if (this->procid == 0) {
                execvp(script, argv);
                exit(0);
            }

            // Open a socket for syncing camera images with the server, and sockets for comms
            this->cameraSyncSocket = SocketClient("localhost", CAMERA_PORT);
            this->cameraSyncSocket.connectToServer();
            this->commsInSocket = SocketClient("localhost", COMMS_IN_PORT);
            this->commsInSocket.connectToServer();
            this->commsOutSocket = SocketClient("localhost", COMMS_OUT_PORT);
            this->commsOutSocket.connectToServer();
        }

        void update(char * imageBytes, int imageWidth, int imageHeight,
                char * requestStr, int & requestLen)
        {
            // Use OpenCV to save image as JPEG
            Mat image = Mat(imageHeight, imageWidth, CV_8UC3, imageBytes);
            flip(image, image, 0);                 // rectify image
            cvtColor(image, image, COLOR_BGR2RGB); // convert image BGR->RGB
            imwrite(IMAGE_TO_PYTHON, image);

            // Send sync byte to Python server, which will open the image, process it, and
            // write the processed image to another file
            char sync = 0;
            this->cameraSyncSocket.send(&sync, 1);

            // If server has created a file for the processed image, open it copy its bytes back to V-REP's camera image
            struct stat fileStat; 
            if (!stat(IMAGE_FROM_PYTHON, &fileStat)) {
                Mat image2 = imread(IMAGE_FROM_PYTHON, CV_LOAD_IMAGE_COLOR);
                flip(image2, image2, 0);                 // rectify image
                cvtColor(image2, image2, COLOR_RGB2BGR); // convert image BGR->RGB
                memcpy(imageBytes, image2.data, imageWidth*imageHeight*3);
            }

            // Check whether bytes are available from server
            int avail = this->commsInSocket.available();

            // Ignore OOB values for available bytes
            if (avail > 0 && avail < MAXMSG) {
                char msg[MAXMSG];
                this->commsInSocket.recv(msg, avail);
                memcpy(requestStr, msg, avail);
                requestLen = avail;
            }
        }

        void sendByte(uint8_t b)
        {
            this->commsOutSocket.send((char *)&b, 1);
        }

        void halt(void)
        {
            if (this->procid) {
                this->cameraSyncSocket.halt();
                this->commsInSocket.halt();
                this->commsOutSocket.halt();
                kill(this->procid, SIGKILL);
            }
        }

}; // CompanionBoard

static CompanionBoard companionBoard;

void extrasStart(void)
{
    companionBoard.start();
}

void extrasUpdate(void)
{
}

void extrasMessage(int message, int * auxiliaryData, void * customData)
{
    // Handle messages from belly camera
    if (message ==  sim_message_eventcallback_openglcameraview && auxiliaryData[2] == 1) {

        // Send in image bytes, get back serial message request
        char request[200];
        int requestLen = 0;
        companionBoard.update((char *)customData, auxiliaryData[0], auxiliaryData[1], request, requestLen);


        // v_repMessage gets called much more frequently than firmware's serial requests, so avoid interrupting
        // request handling
        if (!mspFromServerLen) {
            mspFromServerLen = requestLen;
            mspFromServerIndex = 0;
            memcpy(mspFromServer, request, requestLen);
        }
        // Flag overwrite of original OpenGL image
        auxiliaryData[3] = 1; 
    }
}

void extrasStop(void)
{
    companionBoard.halt();
}

uint8_t Board::serialAvailableBytes(void)
{
    return mspFromServerLen;
}

uint8_t Board::serialReadByte(void)
{
    mspFromServerLen--;
    return mspFromServer[mspFromServerIndex++];
}

void Board::serialWriteByte(uint8_t c)
{
    companionBoard.sendByte(c);
}

bool Board::sonarInit(uint8_t index) 
{
    return false;
}

void Board::sonarUpdate(uint8_t index)
{
}

uint16_t Board::sonarGetDistance(uint8_t index)
{
    return 0;
}
 
