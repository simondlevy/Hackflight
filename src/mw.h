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

// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

extern serialPort_t * telemport;

extern uint8_t useSmallAngle;
extern uint8_t armed;

extern int16_t gyroZero[3];
extern int16_t gyroData[3];
extern int16_t angle[2];
extern int16_t axisPID[3];
extern int16_t rcCommand[4];
extern int16_t debug[4];
extern int16_t gyroADC[3], accADC[3], accSmooth[3], magADC[3];
extern int32_t accSum[3];
extern uint16_t acc_1G;
extern uint32_t accTimeSum;
extern int      accSumCount;
extern uint32_t currentTime;
extern uint32_t previousTime;
extern uint16_t cycleTime;
extern uint16_t calibratingA;
extern uint16_t calibratingG;

extern int32_t baroPressure;
extern int32_t baroPressure2;
extern int32_t baroTemperature;
extern uint32_t baroPressureSum;

extern int32_t setVelocity;
extern uint8_t velocityControl;
extern int32_t errorVelocityI;
extern int32_t vario;
extern int16_t throttleAngleCorrection;
extern int16_t headFreeModeHold;
extern int16_t heading, magHold;
extern int16_t motor[4];
extern int16_t telemTemperature1;      // gyro sensor temperature

#define PITCH_LOOKUP_LENGTH 7
#define THROTTLE_LOOKUP_LENGTH 12
extern int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];   // lookup table for expo & RC rate PITCH+ROLL
extern int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE

extern sensor_t acc;
extern sensor_t gyro;

// State
void imuInit(void);
void computeIMU(void);
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
void getEstimatedAltitude(int32_t * SonarAlt, int32_t * AltPID, int32_t * EstAlt, int32_t * AltHold);

// Output
void mixerInit(void);
void mixerResetMotors(void);
void writeMotors(void);
void writeAllMotors(int16_t mc);
void mixTable(uint16_t * rcData);

// Serial
void serialInit(uint32_t baudrate);
void serialCom(uint16_t * rcData, int32_t SonarAlt, int32_t EstAlt);

// MSP
//void mspInit(rcReadRawDataPtr *callback);
bool mspFrameComplete(void);
void mspFrameRecieve(void);
