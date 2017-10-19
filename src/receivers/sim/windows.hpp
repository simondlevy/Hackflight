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

#include "receiver.hpp"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>

#include <shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")
#include <conio.h>
#define STRCPY strcpy_s

#include <board.hpp>

namespace hf {

    class Controller : public Receiver {

        public:

            bool useSerial(void)
            {
                return true;
            }

            Controller(const char * devname = "/dev/input/js0")
            {
                STRCPY(_devname, devname);

                for (int k=0; k<6; ++k)
                    _axisdir[k] = +1;

                _joyfd = 0;
            }

            void begin(void)
            {
               getProduct();
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

                // Joystick demands are in [-1,+1]; convert to [1000,2000]
                return (uint16_t)(demand*500 + 1500);
            }

            void halt(void)
            {
            }

        private:

            void getProduct(void);

            void poll(void)
            {
                JOYINFOEX joyState;
                joyState.dwSize=sizeof(joyState);
                joyState.dwFlags=JOY_RETURNALL | JOY_RETURNPOVCTS | JOY_RETURNCENTERED | JOY_USEDEADZONE;
                joyGetPosEx(JOYSTICKID1, &joyState);

                /*
                printf(tmp, "%d    X:%d Y:%d Z:%d   R:%d U:%d V:%d  b:%d\n", 
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

                    case SPEKTRUM:
                        _demands[0] =   joynorm(joyState.dwYpos);			// throttle        
                        _demands[1] =   joynorm(joyState.dwZpos);			// roll
                        _demands[2] =   joynorm(joyState.dwVpos);			// pitch
                        _demands[3] =   joynorm(joyState.dwXpos);			// yaw
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
            enum controller_t { TARANIS, SPEKTRUM, EXTREME3D, PS3 , XBOX360};

            controller_t _product;
            char         _devname[100];
            float        _throttleDemand;
            int          _joyfd;
            float        _demands[8];
            uint8_t      _axismap[8];
            int8_t       _axisdir[8];

    };

    void Controller::getProduct(void)
    {
        JOYCAPS joycaps;
        if (joyGetDevCaps(JOYSTICKID1, &joycaps, sizeof(joycaps))==JOYERR_NOERROR) {

            switch (joycaps.wMid) {

                case 3727:
                    _product = PS3;
                    break;

                case 1155:
                    _product = joycaps.wPid == 22288 ? TARANIS : SPEKTRUM;
                    break;

                case 1133:
                    _product = EXTREME3D;
                    break;

                case 9414:
                    _product = XBOX360;
                    break;
            }
        }
    } 

} // namespace
