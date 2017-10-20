/*
   windows.hpp : Windows support for USB controller for flight simulators

   Controller subclasses Receiver

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

namespace hf {    
    
    void Controller::getProduct(uint16_t & vendorId, uint16_t & productId)
    {
        JOYCAPS joycaps;
        if (joyGetDevCaps(JOYSTICKID1, &joycaps, sizeof(joycaps))==JOYERR_NOERROR) {
            vendorId  = joycaps.wMid;
            productId = joycaps.wPid;
        }
    }

    void Controller::pollProduct(void)
    {
        JOYINFOEX joyState;
        joyState.dwSize=sizeof(joyState);
        joyState.dwFlags=JOY_RETURNALL | JOY_RETURNPOVCTS | JOY_RETURNCENTERED | JOY_USEDEADZONE;
        joyGetPosEx(JOYSTICKID1, &joyState);

        // Handle each controller differently
        switch (_vendorId) {

            case VENDOR_STM:
                if (_productId == PRODUCT_TARANIS) {
                    _demands[0] =   joynorm(joyState.dwXpos);			// throttle        
                    _demands[1] =   joynorm(joyState.dwYpos);			// roll
                    _demands[2] =   joynorm(joyState.dwZpos);			// pitch
                    _demands[3] =   joynorm(joyState.dwVpos);			// yaw
                    _demands[4] =   -1;			                        // aux switch
                }
                else { // Spektrum
                    _demands[0] =   joynorm(joyState.dwYpos);			// throttle        
                    _demands[1] =   joynorm(joyState.dwZpos);			// roll
                    _demands[2] =   joynorm(joyState.dwVpos);			// pitch
                    _demands[3] =   joynorm(joyState.dwXpos);			// yaw
                    _demands[4] =   -1;			                        // aux switch
                }
                break;

            case VENDOR_SONY:
                _demands[0] = -joynorm(joyState.dwYpos);            // throttle
                _demands[1] =  joynorm(joyState.dwZpos);            // roll
                _demands[2] = -joynorm(joyState.dwRpos);            // pitch
                _demands[3] =  joynorm(joyState.dwXpos);            // yaw
                //buttonToAuxDemand(_demands, joyState.dwButtons);    // aux switch
                break;

            case VENDOR_MICROSOFT: // XBox 360
                _demands[0] = -joynorm(joyState.dwYpos);            // throttle
                _demands[1] =  joynorm(joyState.dwUpos);            // roll
                _demands[2] = -joynorm(joyState.dwRpos);            // pitch
                _demands[3] =  joynorm(joyState.dwXpos);            // yaw
                //buttonToAuxDemand(_demands, joyState.dwButtons); // aux switch
                break;

            case VENDOR_LOGITECH: // Extreme Pro 3D
                _demands[0] = -joynorm(joyState.dwZpos);            // throttle
                _demands[1] =  joynorm(joyState.dwXpos);            // roll
                _demands[2] = -joynorm(joyState.dwYpos);            // pitch
                _demands[3] =  joynorm(joyState.dwRpos);            // yaw
                //buttonToAuxDemand(_demands, joyState.dwButtons); // aux switch
                break;
        }
    }

} // namespace
