/*
   hover.hpp : Class declaration for hover-in-place functionality

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

#ifdef __arm__
extern "C" {
#else
#include <stdio.h>
#endif

#include "hackflight.hpp"

class Hover {

    private:

        class IMU    * imu;
        class Sonars * sonars;
        class RC     * rc;

        typedef enum {
            MODE_NORMAL,
            MODE_ALTHOLD,
            MODE_GUIDED
        } mode_t;

        mode_t flightMode;

        bool     altHoldChanged;
        int16_t  altHoldCorrection;
        int32_t  altHoldValue;
        uint16_t lastSonarAlt;
        int16_t  altHoldPID;
        int16_t  errorAltitudeI;
        int16_t  initialThrottleHold;
        uint32_t previousT;
        bool     wasArmed;

    public:

        // shared with MSP
        int32_t  estAlt;
        int32_t  vario;

        // for heading
        int16_t headHold; 
  
        // called by MW
        void init(class IMU * _imu, class Sonars * _sonars, class RC * _rc);
        void checkSwitch(void);
        void updateAltitudePid(void);
        void perform(void);
};


#ifdef __arm__
} // extern "C"
#endif
