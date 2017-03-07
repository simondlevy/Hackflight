/*
   hackflight.hpp : general header

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

#include <stdint.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>

#include "crossplatform.h"

#ifndef M_PI
#endif

void debug(const char * format, ...);

#include "board.hpp"
#include "imu.hpp"
#include "rc.hpp"
#include "stabilize.hpp"
#include "mixer.hpp"
#include "msp.hpp"
#include "filters.hpp"

#ifndef abs
#define abs(x)    ((x) > 0 ? (x) : -(x))
#define sgn(x)    ((x) > 0 ? +1 : -1)
#define constrain(val, lo, hi) (val) < (lo) ? lo : ((val) > hi ? hi : val) 
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

// Config =====================================================

#define CONFIG_MAGNETIC_DECLINATION                 0

#define CONFIG_CALIBRATING_ACC_MSEC                 1400

#define CONFIG_YAW_CONTROL_DIRECTION                1    // 1 or -1 
#define CONFIG_RC_LOOPTIME_MSEC                     21
#define CONFIG_CALIBRATE_ACCTIME_MSEC               500
#define CONFIG_SMALL_ANGLE                          250  // tenths of a degree
#define CONFIG_ALTITUDE_UPDATE_MSEC                 25   // based on accelerometer low-pass filter


class TimedTask {

    private:

        uint32_t usec;
        uint32_t period;

    public:

        void init(uint32_t _period) {

            this->period = _period;
            this->usec = 0;
        }

        bool checkAndUpdate(uint32_t currentTime) {

            bool result = (int32_t)(currentTime - this->usec) >= 0;

            if (result)
                this->update(currentTime);

            return result;
        }

        void update(uint32_t currentTime) {

            this->usec = currentTime + this->period;
        }

        bool check(uint32_t currentTime) {

            return (int32_t)(currentTime - this->usec) >= 0;
        }
};


class Hackflight {

    private:

        class IMU        imu;
        class RC         rc;
        class Mixer      mixer;
        class MSP        msp;
        class Stabilize  stab;
        class Board      board;

        class TimedTask imuTask;
        class TimedTask rcTask;
        class TimedTask accelCalibrationTask;

        int16_t  accelADC[3];   // [-4096,+4096]
        int16_t  gyroADC[3];    // [-4096,+4096]
        uint32_t imuLooptimeUsec;
        uint16_t calibratingGyroCycles;
        uint16_t calibratingAccCycles;
        uint16_t calibratingG;
        bool     haveSmallAngle;
        bool     armed;

    public:

        void initialize(void);

        void update(void);
};
