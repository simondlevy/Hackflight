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

#include "controller.hpp"
#include "posix.hpp"

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <linux/joystick.h>

static int joyfd;

controller_t controllerInit(void)
{ 
    controller_t controller = NONE;

    joyfd = open(JOY_DEV, O_RDONLY);

    if (joyfd > 0) {

        fcntl(joyfd, F_SETFL, O_NONBLOCK);

        char name[128];

        if (ioctl(joyfd, JSIOCGNAME(sizeof(name)), name) < 0)
            printf("Uknown controller\n");

        else 
            controller = posixControllerInit(name, "MY-POWER CO.");
    }

    // No joystick detected; use keyboard as fallback
    else  
        posixKbInit();

    return controller;
} 

void controllerRead(controller_t controller, int * demands, void * ignore)
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

    // No joystick; use keyboard
    else  {
        char keys[8] = {68, 67, 66, 65, 50, 10, 54, 53};
        posixKbGrab(keys);
    }
}

void controllerClose(void)
{
    if (joyfd > 0)
        close(joyfd);

    else // reset keyboard if no joystick
        posixKbClose();
}
