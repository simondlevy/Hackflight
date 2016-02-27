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

extern uint8_t useSmallAngle;
extern uint8_t armed;

extern bool    sonarAvailable;
extern int32_t sonarAlt;

extern int16_t gyroZero[3];
extern int16_t gyroData[3];
extern int16_t angle[2];
extern int16_t axisPID[3];
extern int16_t rcCommand[4];
extern int16_t gyroADC[3], accADC[3], accSmooth[3], magADC[3];
extern int32_t accSum[3];
extern uint16_t acc1G;
extern uint32_t currentTime;
extern uint32_t previousTime;
extern uint16_t cycleTime;
extern uint16_t calibratingA;
extern uint16_t calibratingG;

extern bool     baroAvailable;
extern int32_t  baroTemperature;
extern uint32_t baroPressureSum;

extern int32_t estAlt;
extern int32_t altHold;
extern int32_t setVelocity;
extern uint8_t velocityControl;
extern int32_t errorVelocityI;
extern int32_t altPID;
extern int32_t vario;
extern int16_t throttleAngleCorrection;
extern int16_t heading;
extern int16_t motor[4];
extern int16_t rcData[RC_CHANS];

extern sensor_t acc;
extern sensor_t gyro;

// State
void imuInit(void);
void computeIMU(void);
int getEstimatedAltitude();

// Sensors
void initSensors(void);
void ACC_getADC(void);
void Gyro_getADC(void);
bool initBaro(baro_t * baro);
int Baro_update(void);
bool initSonar();
void Sonar_update(void);

// Output
void mixerInit(void);
void writeMotors(void);
void writeAllMotors(int16_t mc);
void mixTable(void);

// Serial
void serialInit(void);
void serialCom(void);

// General
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
