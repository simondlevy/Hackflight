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
#include <netdb.h>
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
    this->pid = 0;
}

void CompanionBoard::start(void)
{
#ifdef __linux

    this->pid = 0;

    char script[200];
    sprintf(script, "%s/hackflight_companion.py", VREP_DIR);
    char *argv[2] = {(char *)script, NULL};

    this->pid = fork();

    if (this->pid == 0) {
        execvp(script, argv);
        exit(0);
    }

    // http://web.eecs.utk.edu/~huangj/cs360/360/notes/Sockets/socketfun.c
    struct sockaddr_in sn;
    struct hostent *he;
    if (!(he = gethostbyname("localhost"))) {
        printf("can't gethostname\n");
    }
    int ok = 0;
    while (!ok) {
        sn.sin_family = AF_INET;
        sn.sin_port  = htons(5000);
        sn.sin_addr.s_addr = *(u_long*)(he->h_addr_list[0]);

        if ((this->sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("socket()");
        }
        ok = (connect(this->sockfd, (struct sockaddr*)&sn, sizeof(sn)) != -1);
        if (!ok) sleep (1);
    }  

#endif

}

static const int BUFSIZE = 100000;
static char buf[BUFSIZE];

void CompanionBoard::update(char * imageBytes, int imageWidth, int imageHeight)
{
    // Use OpenCV to convert image to RGB, and save as JPEG
    Mat image = Mat(imageHeight, imageWidth, CV_8UC3, imageBytes);
    flip(image, image, 0);                      // rectify image
    cvtColor(image, image, COLOR_BGR2RGB); // convert image BGR->RGB
    imwrite("image.jpg", image);

    // Send sync byte to Python client, which will open and process the image
    char sync = 0;
    write(this->sockfd, &sync, 1);
}

void CompanionBoard::halt(void)
{
    if (this->pid) {
        close(this->sockfd);
        kill(this->pid, SIGKILL);
    }
}
