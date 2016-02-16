/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#pragma once

#include "baro.h"


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

// mixer ---------------------------------------------------

extern int16_t motor[4];
extern int16_t rcCommand[4];

void mixerInit(void);
void mixerResetMotors(void);
void writeMotors(void);
void writeAllMotors(int16_t mc);
void mixTable(void);

// mw  -----------------------------------------------------

extern bool     armed;
extern int16_t  axisPID[3];
extern uint32_t currentTime;
extern int16_t  rcData[RC_CHANS];
extern bool     useSmallAngle;

void loop(void);
void setup(void);

// sensors -------------------------------------------------

bool sonar_available;

void initSensors(int hwrev);
void ACC_getADC(void);
void Gyro_getADC(void);
bool initBaro(baro_t * baro);
int  Baro_update(void);
bool initSonar();
void Sonar_update(void);

// serial --------------------------------------------------

serialPort_t * serialInit(uint32_t baudrate);
void           serialCom(void);
void           mspInit(rcReadRawDataPtr *callback);
bool           mspFrameComplete(void);
void           mspFrameRecieve(void);

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
void computeIMU(void);
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
int  getEstimatedAltitude();

