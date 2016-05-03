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

// Globals
EXTERN int16_t  angle[2];
EXTERN float    anglerad[2];
EXTERN uint8_t  armed;
EXTERN int16_t  axisPID[3];
EXTERN uint16_t calibratingA;
EXTERN uint16_t calibratingG;
EXTERN int16_t  gyroADC[3];
EXTERN int16_t  heading;
EXTERN int16_t  motors[4];
EXTERN int16_t  motorsDisarmed[4];
EXTERN int16_t  rcCommand[4];
EXTERN int16_t  rcData[RC_CHANS];

// Mixer
void mixerInit(void);
void mixerWriteMotors(void);

// MSP
void mspCom(void);

// Common
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
int constrainer(int amt, int low, int high);

// Board

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
