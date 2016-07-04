/*
   position.cpp : Position class implementation

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/mixer.c

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

class Position {

    private:

        Board * board;
        IMU   * imu;
        Baro  * baro;
        RC    * rc;

        float    accelAlt;
        float    accelVel;
        float    accZ_old;
        int32_t  altHoldValue;
        bool     altHoldMode;
        int32_t  altPID;
        int32_t  baroAlt;
        int32_t  baroAltBaseline;
        int32_t  baroAlt_offset;
        int32_t  errorVelocityI;
        int32_t  estAlt;
        int32_t  fusedBarosonarAlt;
        int16_t  initialThrottleHold;
        int32_t  lastFusedBarosonarAlt;
        uint32_t previousT;
        int32_t  setVelocity;
        float    sonarTransition;
        bool     velocityControl;
        bool     wasArmed;

    public:

        void init(Board * board, IMU * imu, Baro * baro, RC * rc);

        void update(void);

        void computeAltitude(bool armed);

        void holdAltitude(void);
};


#ifdef __arm__
} // extern "C"
#endif
