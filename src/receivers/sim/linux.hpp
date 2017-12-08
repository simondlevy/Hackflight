/*
   linux.hpp : Linux support for USB controllers

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
            _axismap[4] = 5; // We have to skip to this channel to support Aux on Windows
        }
        else if (strstr(prodname, "Horizon Hobby SPEKTRUM")) {
            _axismap[0] = 1;
            _axismap[1] = 2;
            _axismap[2] = 3;
            _axismap[3] = 0;
            _axismap[4] = 4;
        }
        else if (strstr(prodname, "Extreme 3D")) {
            _axismap[0] = 3;
            _axismap[1] = 0;
            _axismap[2] = 1;
            _axismap[3] = 2;
            _reversedVerticals = true;
            _useButtonForAux = true;
        }
        else if (strstr(prodname, "Generic X-Box pad")) {
            _axismap[0] =  1;
            _axismap[1] =  3;
            _axismap[2] =  4;
            _axismap[3] =  0;
            _reversedVerticals = true;
            _springyThrottle = true;
            _useButtonForAux = true;
        }
        else { // default to PS3
            _axismap[0] = 1;
            _axismap[1] = 2;
            _axismap[2] = 3;
            _axismap[3] = 0;
            _reversedVerticals = true;
            _springyThrottle = true;
            _useButtonForAux = true;
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
