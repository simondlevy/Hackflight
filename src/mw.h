/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#pragma once

#define RC_CHANS    (18)

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

extern uint16_t acc1G;
extern int16_t  accADC[3];
extern int16_t  accSmooth[3];
extern int32_t  altHold;
extern int16_t  angle[2];
uint8_t         armed;
extern int16_t  axisPID[3];
extern uint16_t calibratingA;
extern uint16_t calibratingG;
extern uint16_t cycleTime;
extern int32_t  estAlt;
extern int16_t  gyroADC[3];
extern int16_t  gyroZero[3];
extern int16_t  gyroData[3];
extern int16_t  heading;
extern int16_t  magADC[3];
extern int16_t  motors[4];
extern int16_t  rcCommand[4];
extern int16_t  rcData[RC_CHANS];
extern int16_t  throttleAngleCorrection;
uint8_t         useSmallAngle;
extern int32_t  vario;

extern sensor_t acc;
extern sensor_t gyro;

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
void mspInit(void);
void mspCom(void);

// Common
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);

