/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#pragma once


/*********** RC alias *****************/

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

extern int16_t gyroData[3];
extern int16_t magADC[3];

// Shared
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);

// State

void imuInit(uint16_t acc_1G);

void getEstimatedAltitude(
        int16_t * angle, 
        int32_t * SonarAlt, 
        int32_t * AltPID, 
        int32_t * EstAlt, 
        int32_t * AltHold, 
        int32_t *setVelocity, 
        int32_t * errorVelocityI, 
        int32_t * vario, 
        bool velocityControl, 
        uint32_t baroPressureSum, 
        bool armed);

bool getEstimatedAttitude(
        sensor_t * acc, 
        sensor_t * gyro, 
        int16_t * accSmooth,
        int16_t * angle,
        int16_t * heading, 
        int16_t * throttleAngleCorrection, 
        bool armed);

// Output
void mixerInit(void);
void mixerResetMotors(void);
void writeMotors(int16_t motor[4]);
void mixTable(uint16_t * rcData, int16_t * rcCommand, int16_t * motor, int16_t * axisPID, bool armed);

// MSP
void mspInit(void);
void mspCom(
        int16_t * angle,
        uint16_t * rcData, 
        int16_t * accSmooth,
        int32_t SonarAlt, 
        int32_t EstAlt, 
        int32_t vario, 
        int16_t heading, 
        int16_t * motor,
        uint32_t baroPressureSum, 
        uint16_t cycleTime, 
        bool armed, 
        uint16_t acc_1G);
