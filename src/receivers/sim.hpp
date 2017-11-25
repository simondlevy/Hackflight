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
                _joyid = 0;
            }

            void begin(void)
            {
                // Set up axes based on OS and controller
                productInit();

                // Useful for springy-throttle controllers (XBox, PS3)
                _throttleDemand = -1.f;
            }

            float readChannel(uint8_t chan)
            {
                static float demands[5];

                // Poll on first channel request
                if (chan == 0) {
                    poll(demands);
                }

                // Special handling for throttle
                return (chan == 0) ? _throttleDemand : demands[chan];
            }

            void halt(void)
            {
            }

        private:

            // implemented differently for each OS
            void     productInit(void);
            void     productPoll(int32_t axes[6]);
            int32_t  productGetBaseline(void);

            bool         _reversedVerticals;
            bool         _springyThrottle;
            float        _throttleDemand;
            uint8_t      _axismap[5]; // Thr, Ael, Ele, Rud, Aux
            int          _joyid; // Linux file descriptor or Windows joystick ID

            void poll(float * demands)
            {
                static int32_t axes[6];

                // grab the axis values in an OS-specific way
                productPoll(axes);

                // normalize the axes to demands in [-1,+1]
                for (uint8_t k=0; k<5; ++k) {
                    demands[k] = (axes[_axismap[k]] - productGetBaseline()) / 32767.f;
                }


                // invert throttle, pitch if indicated
                if (_reversedVerticals) {
                    demands[0] = -demands[0];
                    demands[2] = -demands[2];
                }

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

void hf::Controller::productPoll(int32_t axes[6])
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
}

int32_t hf::Controller::productGetBaseline(void)
{
    return 32767;
}

// Linux support -----------------------------------------------------------------------------
#else

#include <unistd.h>
#include <sys/time.h>
#include <linux/joystick.h>

static const char * DEVNAME = "/dev/input/js0";

void hf::Controller::productInit(void)
{
    if ((_joyid=open(DEVNAME, O_RDONLY)) > 0) {

        fcntl(_joyid, F_SETFL, O_NONBLOCK);

        char prodname[128];

        if (ioctl(_joyid, JSIOCGNAME(sizeof(prodname)), prodname) < 0) {
            return;
        }

        if (strstr(prodname, "Taranis") || strstr(prodname, "DeviationTx Deviation GamePad")) {
            _axismap[0] = 0;
            _axismap[1] = 1;
            _axismap[2] = 2;
            _axismap[3] = 3;
            _axismap[4] = 4;
        }
        else if (strstr(prodname, "Horizon Hobby SPEKTRUM")) {
            _axismap[0] = 1;
            _axismap[1] = 2;
            _axismap[2] = 3;
            _axismap[3] = 0;
        }
        else if (strstr(prodname, "Extreme 3D")) {
            _axismap[0] = 3;
            _axismap[1] = 0;
            _axismap[2] = 1;
            _axismap[3] = 2;
            _reversedVerticals = true;
        }
        else if (strstr(prodname, "Generic X-Box pad")) {
            _axismap[0] = 1;
            _axismap[1] = 3;
            _axismap[2] = 4;
            _axismap[3] = 0;
            _springyThrottle = true;
            _reversedVerticals = true;
        }
        else { // default to PS3
            _axismap[0] = 1;
            _axismap[1] = 2;
            _axismap[2] = 3;
            _axismap[3] = 0;
            _springyThrottle = true;
            _reversedVerticals = true;
        }
    }
}

void hf::Controller::productPoll(int32_t axes[6])
{
    if (_joyid <= 0) return;

    struct js_event js;

    read(_joyid, &js, sizeof(struct js_event));

    int jstype = js.type & ~JS_EVENT_INIT;

    if (jstype == JS_EVENT_AXIS)  {

        axes[js.number] = js.value;
    }
}

int32_t hf::Controller::productGetBaseline(void)
{
    return 0;
}

#endif
