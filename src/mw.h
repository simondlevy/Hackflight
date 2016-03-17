/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define RC_CHANS    (18)

#define abs(x) ((x) > 0 ? (x) : -(x))

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

typedef enum {
    ALIGN_DEFAULT = 0,                                      // driver-provided alignment
    CW0_DEG = 1,
    CW90_DEG = 2,
    CW180_DEG = 3,
    CW270_DEG = 4,
    CW0_DEG_FLIP = 5,
    CW90_DEG_FLIP = 6,
    CW180_DEG_FLIP = 7,
    CW270_DEG_FLIP = 8
} sensor_align_e;

#ifndef  EXTERN
#define  EXTERN  extern
#endif

EXTERN uint16_t acc1G;
EXTERN int16_t  accADC[3];
EXTERN int16_t  accSmooth[3];
EXTERN int32_t  altHold;
EXTERN int16_t  angle[2];
EXTERN uint8_t  armed;
EXTERN int16_t  axisPID[3];
EXTERN uint16_t calibratingA;
EXTERN uint16_t calibratingG;
EXTERN uint16_t cycleTime;
EXTERN int32_t  estAlt;
EXTERN int16_t  gyroADC[3];
EXTERN int16_t  gyroZero[3];
EXTERN int16_t  gyroData[3];
EXTERN float    gyroScale;
EXTERN int16_t  heading;
EXTERN int16_t  magADC[3];
EXTERN int16_t  motors[4];
EXTERN int16_t  motor_disarmed[4];
EXTERN int16_t  rcCommand[4];
EXTERN int16_t  rcData[RC_CHANS];
EXTERN int16_t  throttleAngleCorrection;
EXTERN bool     useSmallAngle;
EXTERN int32_t  vario;

// State
void stateInit(void);
void stateComputeAngles(void);

// Sensors
void sensorsInit(void);
void sensorsGetAccel(void);
void sensorsGetGyro(void);

// Mixer
void mixerInit(void);
void mixerWriteMotors(void);

// MSP
void mspCom(void);

// Common
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
int constrain(int amt, int low, int high);
