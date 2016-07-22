/*
   posix.cpp : POSIX (Linux, OS X) support for Hackflight simulator plugin

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

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <string.h>

#include <sys/time.h>

#include "controller.hpp"
#include "posix.hpp"

static int axisdir[5];
static int axismap[5];

static struct termios oldSettings;

void posixKbInit(void)
{
    struct termios newSettings;

    // Save keyboard settings for restoration later
    tcgetattr(fileno( stdin ), &oldSettings);

    // Create new keyboard settings
    newSettings = oldSettings;
    newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(fileno(stdin), TCSANOW, &newSettings);
}

void posixKbGrab(char keys[8])
{
    fd_set set;
    struct timeval tv;

    tv.tv_sec = 0;
    tv.tv_usec = 100;

    FD_ZERO( &set );
    FD_SET( fileno( stdin ), &set );

    int res = select(fileno(stdin )+1, &set, NULL, NULL, &tv);

    char c = 0;

    if (res > 0) {
        read(fileno( stdin ), &c, 1);
        kbRespond(c, keys);
    }

    else if( res < 0 ) 
        perror( "select error" );
}

void posixKbClose(void)
{
    tcsetattr(fileno(stdin), TCSANOW, &oldSettings);
}

controller_t posixControllerInit(char * name, const char * ps3name)
{
    controller_t controller = NONE;

    for (int k=0; k<5; ++k)
        axisdir[k] = +1;

    if (strstr(name, "Taranis")) {
        controller = TARANIS;
        axismap[0] = 0;
        axismap[1] = 1;
        axismap[2] = 2;
        axismap[3] = 3;
        axismap[4] = 4;
    }
    else if (strstr(name, "PPM TO USB Adapter")) {
        controller = SPEKTRUM;
        axismap[0] = 1;
        axismap[1] = 2;
        axismap[2] = 5;
        axismap[3] = 0;
        axismap[4] = 3;
    }
    else if (strstr(name, ps3name)) {
        controller = PS3;
        axismap[0] = 2;
        axismap[1] = 3;
        axismap[2] = 0;
        axismap[3] = 1;
        axisdir[1] = -1;
        axisdir[3] = -1;
    }
    else if (strstr(name, "Extreme 3D")) {
        controller = EXTREME3D;
        axismap[0] = 0;
        axismap[1] = 1;
        axismap[2] = 2;
        axismap[3] = 3;
        axisdir[1] = -1;
        axisdir[3] = -1;
    }
    else {
        printf("Uknown controller: %s\n", name);
    }

    return controller;
}

void posixControllerGrabAxis(controller_t controller, int * demands, int number, int value)
{
    // Look at all five axes for R/C transmitters, first four for other controllers
    int maxaxis = (controller == TARANIS || controller == SPEKTRUM) ? 5 : 4;

    // Grab demands from axes
    for (int k=0; k<maxaxis; ++k)
        if (number == axismap[k]) 
            demands[k] = axisdir[k] * (int)(1000. * value / 32767);
}

void posixControllerGrabButton(int * demands, int number)
{
    switch (number) {
        case 0:
            demands[4] = -1000;
            break;
        case 1:
            demands[4] =     0;
            break;
        case 2:
            demands[4] = +1000;
    }
}

