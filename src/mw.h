/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#pragma once

// RC alias -----------------------------------------------

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

// mw  -----------------------------------------------------

extern int16_t  axisPID[3];
extern uint32_t currentTime;
extern int16_t  rcData[RC_CHANS];
extern bool     useSmallAngle;

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);

// state ---------------------------------------------------

extern uint16_t acc_1G;
extern int32_t  AltHold;
extern int32_t  AltPID;
extern int16_t  angle[2];
extern bool     baro_available;
extern int32_t  baroPressure;
extern uint32_t baroPressureSum;
extern int32_t  baroTemperature;
extern uint16_t calibratingA;
extern uint16_t calibratingG;
extern int32_t  errorVelocityI;
extern int32_t  EstAlt;
extern sensor_t gyro;
extern int16_t  gyroADC[3], accADC[3], accSmooth[3], magADC[3];
extern int16_t  gyroZero[3];
extern int16_t  gyroData[3];
extern int16_t  heading;
extern int32_t  setVelocity;
extern int32_t  SonarAlt;
extern int16_t  throttleAngleCorrection;
extern int32_t  vario;
extern bool     velocityControl;

void imuInit(void);
void computeIMU(bool armed);
int  getEstimatedAltitude(bool armed);

