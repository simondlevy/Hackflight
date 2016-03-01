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

extern int32_t accSum[3];
extern int16_t accADC[3];
extern int16_t accSmooth[3];
extern int32_t altHold;
extern int32_t altPID;
extern int16_t angle[2];
extern int32_t errorVelocityI;
extern int32_t estAlt;
extern int16_t gyroADC[3];
extern int16_t gyroZero[3];
extern int16_t gyroData[3];
extern int16_t magADC[3];
extern int32_t setVelocity;
extern int32_t sonarAlt;
extern int16_t throttleAngleCorrection;
extern int32_t vario;
extern uint8_t velocityControl;

void stateInit(float gyro_scale);
void stateEstimateAngles(void);
void stateEstimateAltitude();

// Sensors

extern uint16_t acc1G;
extern int32_t  baroTemperature;
extern uint32_t baroPressureSum;
extern uint16_t calibratingA;
extern uint16_t calibratingG;
extern int16_t  heading;

void sensorsInit(bool cuttingEdge, bool * baroAvailable, bool * sonarAvailable, float * gyroScale);
void sensorsGetAcc(void);
void sensorsGetGyro(void);
int  sensorsUpdateBaro(void);
void sensorsUpdateSonar(void);

// Mixer

void mixerInit(void);
void mixerWriteMotors(void);
void mixerGetMotors(int16_t * motors_copy);

// Serial
void serialInit(void);
void serialCom(void);

// MW

extern uint8_t  armed;
extern int16_t  axisPID[3];
extern uint32_t currentTime;
extern uint16_t cycleTime;
extern uint8_t  dynP8[3], dynI8[3], dynD8[3];
extern uint32_t previousTime;
extern int16_t  rcCommand[4];
extern int16_t  rcData[RC_CHANS];
extern bool     useSmallAngle;

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
