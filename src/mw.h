/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#pragma once

#define RC_CHANS (8)

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


// State

extern int32_t altHold;
extern int32_t altPID;
extern int16_t imuAngles[2];
extern int32_t errorVelocityI;
extern int32_t setVelocity;
extern int32_t sonarAlt;
extern int16_t throttleAngleCorrection;
extern uint8_t velocityControl;

void stateInit(float gyro_scale);
void stateEstimateAngles(int16_t * gyroOut, bool armed, bool *useSmallAngle);
void stateEstimateAltitude(bool armed, int32_t *estAltOut);
void stateGetRawIMU(int16_t * raw);
void stateGetAttitude(int16_t * headingOut);
void stateGetAltitude(int32_t *estAltOut, int32_t * varioOut);

// Sensors

extern uint16_t acc1G;
extern int16_t  accADC[3];
extern int16_t  gyroADC[3];
extern uint32_t baroPressureSum;
extern uint16_t calibratingA;
extern uint16_t calibratingG;

void sensorsInit(bool cuttingEdge, bool * baroAvailable, bool * sonarAvailable, float * gyroScale);
void sensorsGetAcc(void);
void sensorsGetGyro(void);
int  sensorsUpdateBaro(void);
void sensorsUpdateSonar(void);

// Mixer
void     mixerInit(void);
void     mixerWriteMotors(bool armed);
uint16_t mixerGetMotor(uint8_t i);
void     mixerSetMotor(uint8_t i, uint16_t value);

// Serial

void serialInit(void);
void serialCom(bool armed);

// MW

extern int16_t  axisPID[3];
extern uint32_t currentTime;
extern uint16_t cycleTime;
extern int16_t  rcCommand[4];
extern int16_t  rcData[RC_CHANS];

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
