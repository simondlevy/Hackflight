/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "printf.h"

#define RC_CHANS    (18)

#ifndef abs
#define abs(x) ((x) > 0 ? (x) : -(x))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
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

// Globals ====================================================

EXTERN float    anglerad[2];
EXTERN uint8_t  armed;
EXTERN uint16_t calibratingA;
EXTERN uint16_t calibratingG;
EXTERN int16_t  gyroADC[3];
EXTERN int16_t  heading;

// Board =======================================================

void     board_delayMilliseconds(uint32_t msec);
uint32_t board_getMicros();
void     board_imuComputeAngles(void);
void     board_imuInit(void);
void     board_init(void);
void     board_ledOff(void);
void     board_ledOn(void);
uint16_t board_pwmRead(uint8_t chan);
void     board_writeMotor(uint8_t index, uint16_t value);
uint8_t  board_serialAvailable(void);
uint8_t  board_serialRead(void);
void     board_serialWrite(uint8_t c);

void     board_checkReboot(bool pendReboot);
uint16_t board_getI2cErrorCounter(void);
void     board_reboot(void);

// Config =====================================================

static const uint8_t CONFIG_LEVEL_P  = 90;
static const uint8_t CONFIG_LEVEL_I  = 10;
static const uint8_t CONFIG_LEVEL_D  = 100;

// roll, pitch, yaw
static const uint8_t CONFIG_AXIS_P[3] = {40, 40, 85};
static const uint8_t CONFIG_AXIS_I[3] = {30, 30, 45};
static const uint8_t CONFIG_AXIS_D[3] = {23, 23, 0};

#define CONFIG_MAGNETIC_DECLINATION                 0
#define CONFIG_HORIZON_MODE                         true
#define CONFIG_CALIBRATING_GYRO_CYCLES              1000
#define CONFIG_CALIBRATING_ACC_CYCLES               400
#define CONFIG_MIDRC                                1500
#define CONFIG_MINCOMMAND                           1000
#define CONFIG_GYRO_CMPF_FACTOR                     600    
#define CONFIG_GYRO_CMPFM_FACTOR                    250  
#define CONFIG_MINTHROTTLE                          990
#define CONFIG_MAXTHROTTLE                          2010
#define CONFIG_MINCHECK                             1100
#define CONFIG_YAW_CONTROL_DIRECTION                1   /* 1 or -1 */
#define CONFIG_MAX_ANGLE_INCLINATION                500 /* 50 degrees */
#define CONFIG_MAXCHECK                             1900

#define CONFIG_IMU_LOOPTIME_USEC                    3500
#define CONFIG_RC_LOOPTIME_USEC                     20000
#define CONFIG_CALIBRATE_ACCTIME_USEC               500000

#define CONFIG_MORON_THRESHOLD                      32
#define CONFIG_REBOOT_CHARACTER                     'R'
#define CONFIG_RC_EXPO_8                            65
#define CONFIG_RC_RATE_8                            90
#define CONFIG_THR_MID_8                            50
#define CONFIG_THR_EXPO_8                           0
#define CONFIG_YAW_DIRECTION                        1
#define CONFIG_FAILSAFE_DELAY                       10   /* 1sec */

// the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
#define CONFIG_THROTTLE_CORRECTION_ANGLE            800   

#define CONFIG_YAW_RATE                             0

#define CONFIG_DYN_THR_PID                          0
#define CONFIG_TPA_BREAKPOINT                       1500
#define CONFIG_ACC_LPF_FACTOR                       4
#define CONFIG_ACCZ_DEADBAND                        40
#define CONFIG_ACCXY_DEADBAND                       40
#define CONFIG_ACCZ_LPF_CUTOFF                      5.0F
#define CONFIG_ACC_UNARMEDCAL                       1
#define CONFIG_SMALL_ANGLE                          250 // tenths of a degree
#define CONFIG_DEADBAND                             0
#define CONFIG_YAW_DEADBAND                         0
#define CONFIG_BARO_TAB_SIZE                        21

static const uint8_t CONFIG_ROLL_PITCH_RATE[2] = {0, 0};
static const int16_t CONFIG_ANGLE_TRIM[2]      = {0, 0};
static const uint8_t CONFIG_RCMAP[8]           = {0, 1, 3, 2, 4, 5, 6, 7};
