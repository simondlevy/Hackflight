/*
   sim.hpp : Support USB controller for flight simulators

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

#include "receiver.hpp"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>

#include <board.hpp>

namespace hf {

    class Controller : public Receiver {

        public:

            bool useSerial(void)
            {
                return true;
            }

            Controller(void)
            {
                _reversedVerticals = false;
                _springyThrottle = false;
                _joyfd = 0;
            }

            void begin(void)
            {
                // Set up axes based on OS and controller
                getProduct();

                // Useful for springy-throttle controllers (XBox, PS3)
                _throttleDemand = -1.f;
            }

            uint16_t readChannel(uint8_t chan)
            {
                static float demands[5];

                // Poll on first channel request
                if (chan == 0) {
                    poll(demands);
                }

                // Special handling for throttle
                float demand = (chan == 0) ? _throttleDemand : demands[chan];

                // Joystick demands are in [-1,+1]; convert to [1000,2000]
                return (uint16_t)(demand*500 + 1500);
            }

            void halt(void)
            {
            }

        private:

            // implemented differently for each OS
            void getProduct(void);
            void pollProduct(int32_t axes[6]);

            static const uint16_t VENDOR_SONY      = 0x0e8f;
            static const uint16_t VENDOR_STM       = 0x0483;
            static const uint16_t VENDOR_LOGITECH  = 0x046d;
            static const uint16_t VENDOR_MICROSOFT = 0x24c6;
            static const uint16_t PRODUCT_TARANIS  = 0x5710;

            bool         _reversedVerticals;
            bool         _springyThrottle;
            float        _throttleDemand;
            uint8_t      _axismap[4];
            int          _joyfd; // needed for Linux only

            void poll(float * demands)
            {
                // grab the axis values in an OS-specific way
                int32_t axes[6];
                pollProduct(axes);

                // normalize the axes to demands in [-1,+1]
                for (uint8_t k=0; k<4; ++k) {
                    demands[k] = (axes[_axismap[k]] - (float)32767) / 32767;
                }

                // invert throttle, pitch if indicated
                if (_reversedVerticals) {
                    demands[0] = -demands[0];
                    demands[2] = -demands[2];
                }

                /*
                char tmp[200];
                sprintf_s(tmp, "%d    T: %f    A: %f    E: %f    R: %f\n", 
                        _reversedVerticals, demands[0], demands[1], demands[2], demands[3]);
                OutputDebugStringA(tmp);
                */

                // game-controller spring-mounted throttle requires special handling
                if (_springyThrottle) {
                    if (abs(demands[0]) < .15) {
                        demands[0] = 0; // deadband filter
                    }
                    _throttleDemand += demands[0] * .01f; // XXX need to make this deltaT computable
                    if (_throttleDemand < -1)
                        _throttleDemand = -1;
                    if (_throttleDemand > 1)
                        _throttleDemand = 1;
                }
                else {
                    _throttleDemand = demands[0];
                }
            }

    }; // class Controller

} // namespace hf

// Windows support -----------------------------------------------------------------------------

#ifdef _WIN32

#include <shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")
#include <conio.h>

void hf::Controller::getProduct(void)
{
    JOYCAPS joycaps;
    if (joyGetDevCaps(JOYSTICKID1, &joycaps, sizeof(joycaps))==JOYERR_NOERROR) {

        uint16_t vendorId  = joycaps.wMid;
        uint16_t productId = joycaps.wPid;

        // R/C transmitter
        if (vendorId == VENDOR_STM) {

            if (productId == PRODUCT_TARANIS) {
                _axismap[0] =   0;
                _axismap[1] =   1;
                _axismap[2] =   2;
                _axismap[3] =   3;
            }
            else { // Spektrum
                _axismap[0] =   1;
                _axismap[1] =   2;
                _axismap[2] =   5;
                _axismap[3] =   0;
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
                    break;

                case VENDOR_MICROSOFT: // XBox 360
                    _axismap[0] = 1;
                    _axismap[1] = 4;
                    _axismap[2] = 3;
                    _axismap[3] = 0;
                    _springyThrottle = true;
                    break;

                case VENDOR_LOGITECH:  // Extreme Pro 3D
                    _axismap[0] = 2;
                    _axismap[1] = 0;
                    _axismap[2] = 1;
                    _axismap[3] = 3;
                    break;
            }

        }

    }
}

void hf::Controller::pollProduct(int32_t axes[6])
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


// Linux support -----------------------------------------------------------------------------
#else


#endif

