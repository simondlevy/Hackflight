/*
   controller_Darwin.cpp : OS X support for controllers in Hackflight simulator plugin

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

#include "controller.hpp"
#include "controller_Posix.hpp"

#include <SDL.h>

#include <sys/select.h>

static SDL_Joystick * joystick;

controller_t controllerInit(void)
{
    if (SDL_Init(SDL_INIT_JOYSTICK)) {
        printf("Failed to initialize SDL; using keyboard\n");
        posixKbInit();
        return KEYBOARD;
    }

    if (!(joystick = SDL_JoystickOpen(0))) {
        printf("Unable to open joystick; using keyboard\n");
        posixKbInit();
        return KEYBOARD;
    }

    char name[100];
    strcpy(name, SDL_JoystickNameForIndex(0));

    return posixControllerInit(name, "2In1 USB Joystick");
}

void controllerRead(controller_t controller, int * demands)
{
    if (joystick) {

        // Poll until we get an event
        SDL_Event event;
        while (SDL_PollEvent(&event))
            ;

        if (event.type == SDL_JOYAXISMOTION) {
            SDL_JoyAxisEvent js = event.jaxis;
            posixControllerGrabAxis(controller, demands, js.axis, js.value);
        }

        if (event.type == SDL_JOYBUTTONDOWN) {
            SDL_JoyButtonEvent jb = event.jbutton;
            posixControllerGrabButton(demands, jb.button);
        }
     }

    // Fall back on keyboard if no controller
    else {
        char keys[8] = {52, 54, 50, 56, 48, 10, 51, 57};
        posixKbGrab(keys);
    }
}

void controllerClose(void)
{
    if (joystick)
        SDL_JoystickClose(joystick);

    else // reset keyboard if no joystick
        posixKbClose();
}
