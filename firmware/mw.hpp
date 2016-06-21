/*
   mw.hpp : general header

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

#include <stdint.h>
#include <stdarg.h>

#ifndef _WIN32
#include <stdbool.h>
#endif

#ifdef STM32F10X_MD 
#include <printf.h>
#endif

#include "board.hpp"
#include "imu.hpp"
#include "rc.hpp"
#include "pid.hpp"
#include "mixer.hpp"
#include "msp.hpp"
#include "baro.hpp"
#include "position.hpp"

#ifndef abs
#define abs(x)    ((x) > 0 ? (x) : -(x))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define constrain(val, lo, hi) (val) < (lo) ? lo : ((val) > hi ? hi : val) 
#endif

enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4
};

typedef enum {
    X = 0,
    Y,
    Z
} sensor_axis_e;

#ifndef  EXTERN
#define  EXTERN  extern
#endif

// Config =====================================================

#define CONFIG_MAGNETIC_DECLINATION                 0

#define CONFIG_CALIBRATING_ACC_MSEC                 1400

#define CONFIG_YAW_CONTROL_DIRECTION                1    // 1 or -1 
#define CONFIG_RC_LOOPTIME_MSEC                     20
#define CONFIG_CALIBRATE_ACCTIME_MSEC               500
#define CONFIG_SMALL_ANGLE                          250  // tenths of a degree
#define CONFIG_ALTITUDE_UPDATE_MSEC                 25   // based on accelerometer low-pass filter
