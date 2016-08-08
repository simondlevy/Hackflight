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

#ifdef __arm__
extern "C" {
#else
#include <stdio.h>
#endif

#include "mw.hpp"

class Hover {

    private:

        class IMU   * imu;
        class Baro  * baro;
        class RC    * rc;

        typedef enum {
            MODE_NORMAL,
            MODE_ALTHOLD,
            MODE_GUIDED
        } mode_t;

        mode_t flightMode;

        bool     altHoldChanged;
        int16_t  altHoldCorrection;
        int32_t  altHoldValue;
        int32_t  baroAlt;
        int32_t  baroAltBaseline;
        int16_t  baroPID;
        int16_t  errorAltitudeI;
        int16_t  initialThrottleHold;
        int32_t  lastBaroAlt;
        uint32_t previousT;
        float    vel;
        bool     wasArmed;

        static const uint16_t THROTTLE_NEUTRAL_ZONE  = 40;
        static const float    BARO_CF_VEL            = 0.985f;

    public:

        // shared with MSP
        int32_t  estAlt;
        int32_t  vario; // XXX fixed at zero for now

        // for heading
        int16_t headHold; 
  
        void init(class IMU * _imu, class Baro * _baro, class RC * _rc);

        void checkSwitch(void);

        void updateAltitudePid(bool armed);

        void perform(void);

};


#ifdef __arm__
} // extern "C"
#endif
