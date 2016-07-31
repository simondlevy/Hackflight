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


// Adapted from http://cboard.cprogramming.com/windows-programming/114294-getting-list-usb-devices-listed-system.html
controller_t controllerInit(void)
{ 
    // Get Number Of Devices
    UINT nDevices = 0;
    GetRawInputDeviceList( NULL, &nDevices, sizeof( RAWINPUTDEVICELIST ) );

    // Got Any?
    if(nDevices < 1)
        return KEYBOARD;

    // Allocate Memory For Device List
    PRAWINPUTDEVICELIST pRawInputDeviceList;
    pRawInputDeviceList = new RAWINPUTDEVICELIST[ sizeof( RAWINPUTDEVICELIST ) * nDevices ];

    // Got Memory?
    if( pRawInputDeviceList == NULL ) {
        // Error
        cout << "ERR: Could not allocate memory for Device List.";
        return KEYBOARD;
    }

    // Fill Device List Buffer
    int nResult;
    nResult = GetRawInputDeviceList( pRawInputDeviceList, &nDevices, sizeof( RAWINPUTDEVICELIST ) );

    // Got Device List?
    if( nResult < 0 ) {
        // Clean Up
        delete [] pRawInputDeviceList;

        // Error
        cout << "ERR: Could not get device list.";
        return KEYBOARD;
    }

    // Set Device Info & Buffer Size
    RID_DEVICE_INFO rdiDeviceInfo;
    rdiDeviceInfo.cbSize = sizeof( RID_DEVICE_INFO );
    UINT nBufferSize = rdiDeviceInfo.cbSize;

    // Get Device Info
    nResult = GetRawInputDeviceInfo(pRawInputDeviceList[0].hDevice, RIDI_DEVICEINFO, &rdiDeviceInfo, &nBufferSize );

    // Got All Buffer?
    if(nResult < 0 ) {
        // Error
        cout << "ERR: Unable to read Device Info." << endl;
        return KEYBOARD;
    }

	controller_t controller = KEYBOARD;

    // Some HID
    if (rdiDeviceInfo.dwType == RIM_TYPEHID) {

        switch (rdiDeviceInfo.hid.dwVendorId) {

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

        // XXX could also use if needed: rdiDeviceInfo.hid.dwProductId
    }

    // Clean Up - Free Memory
    delete [] pRawInputDeviceList;

	return controller;
}

// Turns button value into aux-switch demand
static void buttonToAuxDemand(int * demands, int buttons)
{
    if (buttons == 1)
        demands[4] = -1000;

    if (buttons == 2)
        demands[4] = 0;

    if (buttons == 4)
        demands[4] = +1000;
}

static int joynorm(int axisval) 
{
	return (int)((axisval - 32767.)/32767 * 1000);
}

// Grabs stick demands from script via Windows plugin
void controllerRead(controller_t controller, int * demands, void * inDataPtr) 
{
	// Adapted from http://olek.matthewm.com.pl//courses/ee-ces/examples/02_JOYSTICK_WIN32.C.HTML
    JOYINFOEX joyState;
    joyState.dwSize=sizeof(joyState);
    joyState.dwFlags=JOY_RETURNALL | JOY_RETURNPOVCTS | JOY_RETURNCENTERED | JOY_USEDEADZONE;
	joyGetPosEx(JOYSTICKID1, &joyState);

	std::vector<CScriptFunctionDataItem>* inData = (std::vector<CScriptFunctionDataItem>*)inDataPtr;

	/*
	printf("X:%d Y:%d Z:%d   U:%d V:%d R:%d  b:%d\n", 
				joyState.dwXpos, joyState.dwYpos, joyState.dwZpos, 
				joyState.dwUpos, joyState.dwVpos, joyState.dwRpos,
				joyState.dwButtons);
	*/

    // Handle each controller differently
    switch (controller) {

        case TARANIS:
            demands[0] =  joynorm(joyState.dwXpos);		// roll
			demands[1] =  joynorm(joyState.dwYpos);		// pitch
			demands[2] =  joynorm(joyState.dwZpos);		// yaw
			demands[3] =  joynorm(joyState.dwVpos);		// throttle		
            demands[4] = -1000;							// XXX need to map aux switch
            break;

        case SPEKTRUM:
            demands[0] = inData->at(0).int32Data[1];	// roll
            demands[1] = inData->at(0).int32Data[2];	// pitch
            demands[2] = inData->at(1).int32Data[2];	// yaw
            demands[3] = inData->at(0).int32Data[0];	// throttle
            demands[4] = inData->at(1).int32Data[0];    // aux switch
            break;

        case EXTREME3D:
            demands[0] =  inData->at(0).int32Data[0];	// roll
            demands[1] = -inData->at(0).int32Data[1];	// pitch
            demands[2] =  inData->at(1).int32Data[2];	// yaw
            demands[3] = -inData->at(2).int32Data[0];	// throttle
            //buttonToAuxDemand(demands, inData);		    // aux switch
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
