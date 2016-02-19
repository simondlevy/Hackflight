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

extern int16_t gyroZero[3];
extern int16_t gyroData[3];
extern int16_t angle[2];
extern int16_t axisPID[3];
extern int16_t rcCommand[4];
extern uint16_t calibratingA;
extern uint16_t calibratingG;
extern int16_t  gyroADC[3];
extern int16_t  accADC[3];
extern int16_t  magADC[3];
extern int16_t  accSmooth[3];

// Shared
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);

// State
void imuInit(uint16_t acc_1G);
bool getEstimatedAttitude(uint16_t acc_1G, int16_t * heading, sensor_t * gyro, int16_t * throttleAngleCorrection, bool armed);
void getEstimatedAltitude(int32_t * SonarAlt, int32_t * AltPID, int32_t * EstAlt, int32_t * AltHold, 
        int32_t *setVelocity, int32_t * errorVelocityI, int32_t * vario, bool velocityControl, 
        uint32_t baroPressureSum, bool armed);

// Output
void mixerInit(void);
void mixerResetMotors(void);
void writeMotors(int16_t motor[4]);
void mixTable(uint16_t * rcData, int16_t * motor, bool armed);

// MSP
void mspInit();
void mspCom(uint16_t * rcData, int32_t SonarAlt, int32_t EstAlt, int32_t vario, int16_t heading, int16_t * motor,
        uint32_t baroPressureSum, uint16_t cycleTime, bool armed, uint16_t acc_1G);
