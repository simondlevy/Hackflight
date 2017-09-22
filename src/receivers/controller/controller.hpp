/*
   controller.hpp : USB controller support for Hackflight

   Controller class subclasses Receiver

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

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>
#include <linux/joystick.h>

namespace hf {

    class Controller : protected Receiver {

        public:

            bool useSerial(void)
            {
                return true;
            }

            Controller(const char * devname = "/dev/input/js0")
            {
                strcpy(_devname, devname);

                for (int k=0; k<6; ++k)
                    _axisdir[k] = +1;
            }

            void begin(void)
            {
                if ((_joyfd=open(_devname, O_RDONLY)) > 0) {

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
                    else if (strstr(prodname, "PS3")) {
                        _product = PS3;
                        _axismap[0] = 1;
                        _axismap[1] = 2;
                        _axismap[2] = 3;
                        _axismap[3] = 0;
                        _axisdir[0] = -1;
                        _axisdir[1] = -1;
                    }
                    else if (strstr(prodname, "Extreme 3D")) {
                        _product = EXTREME3D;
                        _axismap[0] = 3;
                        _axismap[1] = 0;
                        _axismap[2] = 1;
                        _axismap[3] = 2;
                        _axisdir[0] = -1;
                        _axisdir[1] = -1;
                    }
                    else if (strstr(prodname, "Generic X-Box pad")) {
                        _product = XBOX360;
                        _axismap[0] = 1; 
                        _axismap[1] = 3; 
                        _axismap[2] = 4; 
                        _axismap[3] = 0; 
                        _axisdir[0] = -1;
                        _axisdir[1] = -1;
                    }
                    else {
                        printf("Uknown controller: %s\n", prodname);
                    }
                }

                _throttleDemand = -1.f;
            }

            uint16_t readChannel(uint8_t chan)
            {
                // Special handling for throttle
                float demand = (chan == 0) ? _throttleDemand : _demands[chan];

                // Special handling for pitch, roll on PS3, XBOX360
                if (chan == 1 || chan == 2) {
                    if (_product == PS3)
                        demand /= 2;
                    if (_product == XBOX360)
                        demand /= 1.5;
                }

                // Joystick demands are in [-1,+1]; convert to [1000,2000]
                return (uint16_t)(demand*500 + 1500);
            }

            void update(void)
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

                // game-controller spring-mounted throttle requires special handling
                switch (_product) {
                    case PS3:
                    case XBOX360:
                        _throttleDemand += _demands[0] * .01f;
                        if (_throttleDemand < -1)
                            _throttleDemand = -1;
                        if (_throttleDemand > 1)
                            _throttleDemand = 1;
                        break;
                    default:
                        _throttleDemand = _demands[0];
                }

            }

            void halt(void)
            {
                if (_joyfd > 0)
                    close(_joyfd);
            }

        private:

            // We currently support these controllers
            enum controller_t { TARANIS, EXTREME3D, PS3 , XBOX360};

            controller_t _product;
            char         _devname[100];
            float        _throttleDemand;
            int          _joyfd;
            float        _demands[8];
            uint8_t      _axismap[8];
            int8_t       _axisdir[8];

    };

} // namespace


