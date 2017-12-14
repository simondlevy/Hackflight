/*
   windows.hpp : Windows support for USB controllers

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

#pragma once

#include "sim.hpp"

#include <shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")
#include <conio.h>

static const uint16_t VENDOR_SONY      = 0x0e8f;
static const uint16_t VENDOR_STM       = 0x0483;
static const uint16_t VENDOR_LOGITECH  = 0x046d;
static const uint16_t VENDOR_MICROSOFT = 0x24c6;
static const uint16_t PRODUCT_TARANIS  = 0x5710;

void hf::Controller::productInit(void)
{
    JOYCAPS joycaps;

	// Grab the first available joystick
	for (_joyid=0; _joyid<16; _joyid++)
		if (joyGetDevCaps(_joyid, &joycaps, sizeof(joycaps)) == JOYERR_NOERROR)
			break;


    if (_joyid < 16) {

        uint16_t vendorId  = joycaps.wMid;
        uint16_t productId = joycaps.wPid;

        // axes: 0=Thr 1=Ael 2=Ele 3=Rud 4=Aux
        // JOYINFOEX: 0=dwXpos 1=dwYpos 2=dwZpos 3=dwRpos 4=dwUpos 5=dwVpos

        // R/C transmitter
        if (vendorId == VENDOR_STM) {

            if (productId == PRODUCT_TARANIS) {
                _axismap[0] =   0;
                _axismap[1] =   1;
                _axismap[2] =   2;
                _axismap[3] =   5;
                _axismap[4] =   3;
            }
            else { // Spektrum
                _axismap[0] =   1;
                _axismap[1] =   2;
                _axismap[2] =   5;
                _axismap[3] =   0;
                _axismap[4] =   4;
            }
        }

        else {

            _reversedVerticals = true;

            switch (vendorId) {

                case VENDOR_SONY:      // PS3
                    _axismap[0] = 1;
                    _axismap[1] = 2;
                    _axismap[2] = 3;
                    _axismap[3] = 0;
                    _springyThrottle = true;
                    _useButtonForAux = true;
                    break;

                case VENDOR_MICROSOFT: // XBox 360
                    _axismap[0] = 1;
                    _axismap[1] = 4;
                    _axismap[2] = 3;
                    _axismap[3] = 0;
                    _springyThrottle = true;
                    _useButtonForAux = true;
                    break;

                case VENDOR_LOGITECH:  // Extreme Pro 3D
                    _axismap[0] = 2;
                    _axismap[1] = 0;
                    _axismap[2] = 1;
                    _axismap[3] = 3;
                    _useButtonForAux = true;
                    break;
            }

        }
    }
}

void hf::Controller::productPoll(int32_t axes[6], uint8_t & buttons)
{
    JOYINFOEX joyState;
    joyState.dwSize=sizeof(joyState);
    joyState.dwFlags=JOY_RETURNALL | JOY_RETURNPOVCTS | JOY_RETURNCENTERED | JOY_USEDEADZONE;
    joyGetPosEx(_joyid, &joyState);

    axes[0] = joyState.dwXpos;
    axes[1] = joyState.dwYpos;
    axes[2] = joyState.dwZpos;
    axes[3] = joyState.dwRpos;
    axes[4] = joyState.dwUpos;
    axes[5] = joyState.dwVpos;

    buttons = joyState.dwButtons;
}

int32_t hf::Controller::productGetBaseline(void)
{
    return 32767;
}
