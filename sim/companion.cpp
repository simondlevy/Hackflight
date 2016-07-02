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
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

#include <iostream>
using namespace std;

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h> 
#endif

CompanionBoard::CompanionBoard(void)
{
}

void CompanionBoard::start(void)
{
    this->pid = 0;

#ifdef __linux
    char script[200];
    sprintf(script, "%s/hackflight_companion.py", VREP_DIR);
    char *argv[2] = {(char *)script, NULL};

    this->pid = fork();

    if (this->pid == 0) {
        execvp(script, argv);
        exit(0);
    }

    printf("ready to serve!\n");
    struct sockaddr_in serv_addr; 

    char sendBuff[1025];
    time_t ticks; 

    int listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(sendBuff, '0', sizeof(sendBuff)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(5000); 

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)); 

    listen(listenfd, 10);

    this->sockfd = accept(listenfd, (struct sockaddr*)NULL, NULL); 

    printf("Accepted\n");

#endif
}

void CompanionBoard::update(char * imageBytes, int imageWidth, int imageHeight)
{
    Mat image = Mat(imageHeight, imageWidth, CV_8UC3, imageBytes);
    flip(image, image, 0);                      // rectify image
    cvtColor(image, image, COLOR_BGR2RGB); // convert image BGR->RGB
    imwrite("image.jpg", image);
    static int count;
    write(this->sockfd, &count, 4);
    count++;
    //namedWindow( "OpenCV", WINDOW_AUTOSIZE );   // Create a window for display.
    //imshow( "OpenCV", image );                  // Show our image inside it.
    //waitKey(1);     
}

void CompanionBoard::halt(void)
{
#ifdef __linux
    if (this->pid) {
        close(this->sockfd);
        kill(this->pid, SIGKILL);
    }
#endif
}
