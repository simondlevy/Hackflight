/*
   linux.hpp : Linux support for USB controller for flight simulators

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

#include <unistd.h>
#include <sys/time.h>
#include <linux/joystick.h>

static const char * DEVNAME = "/dev/input/js0";

namespace hf {

    void Controller::getProduct(void)
    {
        if ((_joyfd=open(DEVNAME, O_RDONLY)) > 0) {

            fcntl(_joyfd, F_SETFL, O_NONBLOCK);

            char prodname[128];

            if (ioctl(_joyfd, JSIOCGNAME(sizeof(prodname)), prodname) < 0) {
                printf("Uknown controller\n");
                return;
            }

            if (strstr(prodname, "Taranis") || strstr(prodname, "DeviationTx Deviation GamePad")) {
                _product = TARANIS;
                _axismap[0] = 0;
                _axismap[1] = 1;
                _axismap[2] = 2;
                _axismap[3] = 3;
                _axismap[4] = 4;
            }
            else if (strstr(prodname, "Extreme 3D")) {
                _product = EXTREME3D;
                _axismap[0] = 3;
                _axismap[1] = 0;
                _axismap[2] = 1;
                _axismap[3] = 2;
                _axisdir[0] = -1;
                _axisdir[2] = -1;
            }
            else if (strstr(prodname, "Generic X-Box pad")) {
                _product = XBOX360;
                _axismap[0] = 1; 
                _axismap[1] = 3; 
                _axismap[2] = 4; 
                _axismap[3] = 0; 
                _axisdir[0] = -1;
                _axisdir[2] = -1;
            }
            else { // default to PS3
                _product = PS3;
                _axismap[0] = 1;
                _axismap[1] = 2;
                _axismap[2] = 3;
                _axismap[3] = 0;
                _axisdir[0] = -1;
                _axisdir[2] = -1;
            }
        }
    }

    void Controller::pollProduct(void)
    {
        if (_joyfd <= 0) return;

        struct js_event js;

        read(_joyfd, &js, sizeof(struct js_event));

        int jstype = js.type & ~JS_EVENT_INIT;

        // Grab demands from axes
        if (jstype == JS_EVENT_AXIS)  {

            // Look at all five axes for R/C transmitters, first four for other controllers
            int maxaxis = (_product == TARANIS) ? 5 : 4;

            // Grab demands from axes
            for (int k=0; k<maxaxis; ++k) {
                if (js.number == _axismap[k]) {
                    _demands[k] = _axisdir[k] * js.value / 32767.;
                }
            }
        }
    }

} // namespace hf
