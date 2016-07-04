/*
   hover.hpp : Class declaration for hover-in-place

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

class Hover {

    private:

        RC * rc;
        Position * position;

        float    accelZ_prev;
        int32_t  altHoldValue;
        bool     altHoldMode;
        int32_t  altPID;
        int32_t  errorVelocityI;
        int16_t  initialThrottleHold;
        int32_t  setVelocity;
        bool     velocityControl;
 
    public:

        void init(RC * rc, Position * position);

        void checkSwitch(void);

        void updatePid(void);

        void holdAltitude(void);
};


#ifdef __arm__
} // extern "C"
#endif
