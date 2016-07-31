/*
   controller_Windows.cpp : WIN32 support for controllers in Hackflight simulator plugin

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

#include "v_repExt.h"
#include "scriptFunctionData.h"
#include "v_repLib.h"

#include <shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")

#include <conio.h>

#include <iostream>
using namespace std;

// Adapted from http://olek.matthewm.com.pl//courses/ee-ces/examples/02_JOYSTICK_WIN32.C.HTML

controller_t controllerInit(void)
{ 
	controller_t controller = KEYBOARD;

   JOYCAPS joycaps;
   if (joyGetDevCaps(JOYSTICKID1, &joycaps, sizeof(joycaps))==JOYERR_NOERROR)
       switch (joycaps.wMid) {

            case 3727:
                controller = PS3;
                break;

            case 1155:
                controller = TARANIS;
                break;

            case 1783:
                controller = SPEKTRUM;
                break;

            case 1133:
                controller = EXTREME3D; // XXX product ID = 49685
                break;
        }
 
	return controller;
}

// Turns button value into aux-switch demand
static void buttonToAuxDemand(float * demands, int buttons)
{
    if (buttons == 1)
        demands[4] = -1;

    if (buttons == 2)
        demands[4] = 0;

    if (buttons == 4)
        demands[4] = +1;
}

static float joynorm(int axisval) 
{
	return (axisval - (float)32767) / 32767;
}


void controllerRead(controller_t controller, float * demands) 
{
    JOYINFOEX joyState;
    joyState.dwSize=sizeof(joyState);
    joyState.dwFlags=JOY_RETURNALL | JOY_RETURNPOVCTS | JOY_RETURNCENTERED | JOY_USEDEADZONE;
	joyGetPosEx(JOYSTICKID1, &joyState);
	
	/*
	printf("X:%d Y:%d Z:%d   U:%d V:%d R:%d  b:%d\t", 
				joyState.dwXpos, joyState.dwYpos, joyState.dwZpos, 
				joyState.dwUpos, joyState.dwVpos, joyState.dwRpos,
				joyState.dwButtons);
	*/

	//printf("%d\n", (int)controller);

    // Handle each controller differently
    switch (controller) {

        case TARANIS:
            demands[0] =  joynorm(joyState.dwXpos);		// roll
			demands[1] =  joynorm(joyState.dwYpos);		// pitch
			demands[2] =  joynorm(joyState.dwZpos);		// yaw
			demands[3] =  joynorm(joyState.dwVpos);		// throttle		
            demands[4] = -1;							// XXX need to map aux switch
            break;

        case SPEKTRUM:
            demands[0] =  joynorm(joyState.dwYpos);		// roll
			demands[1] =  joynorm(joyState.dwZpos);		// pitch
			demands[2] =  joynorm(joyState.dwRpos);		// yaw
			demands[3] =  joynorm(joyState.dwXpos);		// throttle		
            demands[4] =  joynorm(joyState.dwVpos);		// aux switch		
            break;

        case EXTREME3D:
            demands[0] =  joynorm(joyState.dwXpos);			// roll
			demands[1] = -joynorm(joyState.dwYpos);			// pitch
			demands[2] =  joynorm(joyState.dwRpos);			// yaw
			demands[3] = -joynorm(joyState.dwZpos);			// throttle
			buttonToAuxDemand(demands, joyState.dwButtons); // aux switch
            break;

        case PS3:
            demands[0] =  joynorm(joyState.dwZpos);			// roll
			demands[1] = -joynorm(joyState.dwRpos);			// pitch
			demands[2] =  joynorm(joyState.dwXpos);			// yaw
			demands[3] = -joynorm(joyState.dwYpos);			// throttle
			buttonToAuxDemand(demands, joyState.dwButtons); // aux switch
            break;

        default:
            if (_kbhit()) {
                char c = _getch();
                char keys[8] = {52, 54, 50, 56, 48, 13, 51, 57};
                kbRespond(c, keys);
            }
    }
}

void controllerClose(void)
{
    // XXX no action needed (?)
}
