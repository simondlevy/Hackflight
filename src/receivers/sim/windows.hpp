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

    void Controller::pollProduct(int32_t axes[6])
    {
        JOYINFOEX joyState;
        joyState.dwSize=sizeof(joyState);
        joyState.dwFlags=JOY_RETURNALL | JOY_RETURNPOVCTS | JOY_RETURNCENTERED | JOY_USEDEADZONE;
        joyGetPosEx(JOYSTICKID1, &joyState);

        axes[0] = joyState.dwXpos;
        axes[1] = joyState.dwYpos;
        axes[2] = joyState.dwZpos;
        axes[3] = joyState.dwRpos;
        axes[4] = joyState.dwUpos;
        axes[5] = joyState.dwVpos;
    }

} // namespace
