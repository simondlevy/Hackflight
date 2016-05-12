/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "printf.h"

#define RC_CHANS    (8)

#include "board.hpp"
#include "imu.hpp"
#include "rc.hpp"
#include "pid.hpp"
#include "mixer.hpp"
#include "msp.hpp"


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
#define CONFIG_CALIBRATING_GYRO_CYCLES              1000
#define CONFIG_CALIBRATING_ACC_CYCLES               400
#define CONFIG_MAXCHECK                             1900
#define CONFIG_MINTHROTTLE                          990
#define CONFIG_MAXTHROTTLE                          2010
#define CONFIG_YAW_CONTROL_DIRECTION                1   /* 1 or -1 */
#define CONFIG_IMU_LOOPTIME_USEC                    3500
#define CONFIG_RC_LOOPTIME_USEC                     20000
#define CONFIG_CALIBRATE_ACCTIME_USEC               500000
#define CONFIG_SMALL_ANGLE                          250 // tenths of a degree
#define CONFIG_BARO_TAB_SIZE                        21

static const uint8_t CONFIG_RCMAP[8]           = {0, 1, 3, 2, 4, 5, 6, 7};
