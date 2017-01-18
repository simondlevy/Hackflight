/*
   controller_Linux.cpp : Linux support for controllers in Hackflight simulator plugin

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

static const char * JOY_DEV = "/dev/input/js0";
static const char * DSM_DEV = "/dev/ttyACM0";

#include "controller.hpp"
#include "controller_Posix.hpp"

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include <linux/joystick.h>

static int joyfd;

static int dsmfd;
static int dsmvals[5];

static void * dsmthread(void * v)
{
    int count = 0;
    while (1)
        printf("%d\n", count++);
}

controller_t controllerInit(void)
{ 
    // Default to keyboard controller
    controller_t controller = KEYBOARD;

    // First try to open wired joystick device
    if ((joyfd=open(JOY_DEV, O_RDONLY)) > 0) {

        fcntl(joyfd, F_SETFL, O_NONBLOCK);

        char name[128];

        if (ioctl(joyfd, JSIOCGNAME(sizeof(name)), name) < 0)
            printf("Uknown controller\n");

        else 
            controller = posixControllerInit(name, "MY-POWER CO.");
    }

    // Next try to open wireless DSM dongle
    else if ((dsmfd=open(DSM_DEV, O_RDONLY)) > 0) {

        pthread_t thread;
        pthread_create(&thread, NULL, dsmthread, NULL);
        controller = DSM;
    }

    // No joystick or dongle detected; use keyboard as fallback
    else  {
        posixKbInit();
    }


    return controller;
} 

void controllerRead(controller_t controller, float * demands)
{
    // Have a joystick; grab its axes
    if (joyfd > 0) {

        struct js_event js;

        read(joyfd, &js, sizeof(struct js_event));

        int jstype = js.type & ~JS_EVENT_INIT;

        // Grab demands from axes
        if (jstype == JS_EVENT_AXIS) 
            posixControllerGrabAxis(controller, demands, js.number, js.value);

        // Grab aux demand from buttons when detected
        if ((jstype == JS_EVENT_BUTTON) && (js.value==1)) 
            posixControllerGrabButton(demands, js.number);
    }

    // No joystick; try DSM dongle
    else if (dsmfd > 0) {
        for (int k=0; k<5; ++k)
            demands[k] = 0;
    }

    // No joystick or DSM; use keyboard
    else  {
        char keys[8] = {68, 67, 66, 65, 50, 10, 54, 53};
        posixKbGrab(keys);
    }
}

void controllerClose(void)
{
    if (joyfd > 0)
        close(joyfd);

    else if (dsmfd > 0)
        close(dsmfd);

    else // reset keyboard if no joystick or DSM dongle
        posixKbClose();
}
