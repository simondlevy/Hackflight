/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#pragma once

// RC alias -----------------------------------------------

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

extern void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);

// msp -----------------------------------------------------

extern void mspCom(bool armed, int16_t * rcData, int16_t motor[4], int16_t motor_disarmed[4], 
        uint16_t acc_1G, int16_t angle[2]);

// state ---------------------------------------------------

extern int32_t  baroPressure;
extern uint32_t baroPressureSum;
extern int32_t  baroTemperature;

extern int32_t  EstAlt;
extern int32_t  SonarAlt;

extern int32_t  errorVelocityI;
extern sensor_t gyro;
extern int16_t  gyroADC[3], accADC[3], accSmooth[3], magADC[3];
extern int16_t  gyroZero[3];
extern int16_t  gyroData[3];
extern int16_t  heading;
extern int32_t  setVelocity;
extern int16_t  throttleAngleCorrection;
extern int32_t  vario;
extern bool     velocityControl;

void    imuInit(uint16_t acc_1G);
bool    computeIMU(bool armed, uint16_t acc_1G, int16_t angle[2], uint16_t * calibratingA, uint16_t * calibratingG);
int32_t getAltPID(bool armed, int32_t AltHold, int16_t angle[2]);
