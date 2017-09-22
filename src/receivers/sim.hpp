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

#ifdef _WIN32

#include <shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")
#include <conio.h>

#else
#include <unistd.h>
#include <sys/time.h>
#include <linux/joystick.h>
#endif

namespace hf {

    class Controller : public Receiver {

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

                _joyfd = 0;
            }

            void begin(void)
            {
#ifdef _WIN32
                JOYCAPS joycaps;
                if (joyGetDevCaps(JOYSTICKID1, &joycaps, sizeof(joycaps))==JOYERR_NOERROR) {

                    switch (joycaps.wMid) {

                        case 3727:
                            _product = PS3;
                            break;

                        case 1155:
                            _product = TARANIS;
                            break;

                        case 1133:
                            _product = EXTREME3D;
                            break;

                        case 9414:
                            _product = XBOX360;
                            break;
                    }
                }

#else
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
#endif
                _throttleDemand = -1.f;
            }

            uint16_t readChannel(uint8_t chan)
            {
                // Poll on first channel request
                if (chan == 0) {
                    poll();
                }

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

            void halt(void)
            {
#ifndef _WIN32
                if (_joyfd > 0)
                    close(_joyfd);
#endif
            }

        private:

            void poll(void)
            {
#ifdef _WIN32
                JOYINFOEX joyState;
                joyState.dwSize=sizeof(joyState);
                joyState.dwFlags=JOY_RETURNALL | JOY_RETURNPOVCTS | JOY_RETURNCENTERED | JOY_USEDEADZONE;
                joyGetPosEx(JOYSTICKID1, &joyState);

                /*
                printf("%d    X:%d Y:%d Z:%d   R:%d U:%d V:%d  b:%d\n", 
                        _product,
                        joyState.dwXpos, joyState.dwYpos, joyState.dwZpos, 
                        joyState.dwRpos, joyState.dwUpos, joyState.dwVpos,
                        joyState.dwButtons);*/

                // Handle each controller differently
                switch (_product) {

                    case TARANIS:
                        _demands[0] =   joynorm(joyState.dwXpos);			// throttle        
                        _demands[1] =   joynorm(joyState.dwYpos);			// roll
                        _demands[2] =   joynorm(joyState.dwZpos);			// pitch
                        _demands[3] =   joynorm(joyState.dwVpos);			// yaw
                        _demands[4] =   -1;			                        // aux switch
                        break;

                    case PS3:
                        _demands[0] = -joynorm(joyState.dwYpos);            // throttle
                        _demands[1] =  joynorm(joyState.dwZpos);            // roll
                        _demands[2] = -joynorm(joyState.dwRpos);            // pitch
                        _demands[3] =  joynorm(joyState.dwXpos);            // yaw
                        //buttonToAuxDemand(_demands, joyState.dwButtons);    // aux switch
                        break;

                    case XBOX360:
                        _demands[0] = -joynorm(joyState.dwYpos);            // throttle
                        _demands[1] =  joynorm(joyState.dwUpos);            // roll
                        _demands[2] = -joynorm(joyState.dwRpos);            // pitch
                        _demands[3] =  joynorm(joyState.dwXpos);            // yaw
                        //buttonToAuxDemand(_demands, joyState.dwButtons); // aux switch
                        break;

                    case EXTREME3D:
                        _demands[0] = -joynorm(joyState.dwZpos);            // throttle
                        _demands[1] =  joynorm(joyState.dwXpos);            // roll
                        _demands[2] = -joynorm(joyState.dwYpos);            // pitch
                        _demands[3] =  joynorm(joyState.dwRpos);            // yaw
                        //buttonToAuxDemand(_demands, joyState.dwButtons); // aux switch
                        break;
                }
#else
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
#endif
                // game-controller spring-mounted throttle requires special handling
                switch (_product) {
                    case PS3:
                    case XBOX360:
                        if (abs(_demands[0]) < .15) {
                            _demands[0] = 0; // deadband filter
                        }
                        _throttleDemand += _demands[0] * .01f; // XXX need to make this deltaT computable
                        if (_throttleDemand < -1)
                            _throttleDemand = -1;
                        if (_throttleDemand > 1)
                            _throttleDemand = 1;
                        break;
                    default:
                        _throttleDemand = _demands[0];
                }
            }

            static float joynorm(int axisval) 
            {
                return (axisval - (float)32767) / 32767;
            }

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
