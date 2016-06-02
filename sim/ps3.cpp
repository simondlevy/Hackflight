/*
   Prototype PS3 controller code for HackflightSim

   Copyright (C) Simon D. Levy 2016

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

#include <iostream>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define JOY_DEV "/dev/input/js1"

// Joystick support
static int joyfd;
static double rollDemand;
static double pitchDemand;
static double yawDemand;
static double throttleDemand;

static float min(float a, float b) {
    return a < b ? a : b;
}

static float max(float a, float b) {
    return a > b ? a : b;
}

int main(int argc, char ** argv)
{
    static float chanvals[4];
    // Initialize joystick
    joyfd = open( JOY_DEV , O_RDONLY);
    if(joyfd > 0) 
        fcntl(joyfd, F_SETFL, O_NONBLOCK);

    static float pitchDemand, rollDemand, yawDemand, throttleDemand;

    while (true) {
        if (joyfd > 0) {
            struct js_event js;
            read(joyfd, &js, sizeof(struct js_event));
            if (js.type & ~JS_EVENT_INIT) {
                float x = js.value / 32767.;
                switch (js.number) {
                    case 0:
                        yawDemand = -x;
                        break;
                    case 1:
                        throttleDemand -= x / 100000;
                        throttleDemand = max(0, min(1, throttleDemand));
                        break;
                    case 2:
                        rollDemand = x;
                        break;
                    case 3:
                        pitchDemand = x;
                }
            }
        }

        printf("%f %f %f %f\n", pitchDemand, rollDemand, yawDemand, throttleDemand);
    }
}

