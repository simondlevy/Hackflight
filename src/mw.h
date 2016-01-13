/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#pragma once

#define BARO_TAB_SIZE_MAX   48

#define RC_CHANS    (18)

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

#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))

// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

extern serialPort_t * telemport;

uint8_t useSmallAngle;
uint8_t armed;

extern bool px4flow_available;
px4flow_frame_t px4flow_frame;

extern bool lidarlite_available;
uint16_t lidarlite_distance;

extern bool sonar_available;
extern int32_t SonarAlt;

extern int16_t gyroZero[3];
extern int16_t gyroData[3];
extern int16_t angle[2];
extern int16_t axisPID[3];
extern int16_t rcCommand[4];
extern int16_t failsafeCnt;
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
extern uint16_t calibratingB;
extern uint16_t calibratingG;

extern bool baro_available;
extern int32_t baroPressure;
extern int32_t baroTemperature;
extern uint32_t baroPressureSum;
extern int32_t BaroAlt;

extern int32_t EstAlt;
extern int32_t AltHold;
extern int32_t setVelocity;
extern uint8_t velocityControl;
extern int32_t errorVelocityI;
extern int32_t BaroPID;
extern int32_t SonarPID;
extern int32_t vario;
extern int16_t throttleAngleCorrection;
extern int16_t headFreeModeHold;
extern int16_t heading, magHold;
extern int16_t motor[4];
extern int16_t rcData[RC_CHANS];
extern int16_t telemTemperature1;      // gyro sensor temperature

#define PITCH_LOOKUP_LENGTH 7
#define THROTTLE_LOOKUP_LENGTH 12
extern int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];   // lookup table for expo & RC rate PITCH+ROLL
extern int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE

extern sensor_t acc;
extern sensor_t gyro;

// main
void loop(void);

// State
void imuInit(void);
void computeIMU(void);
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
int getEstimatedAltitude();

// Sensors
void initSensors(void);
void ACC_getADC(void);
void Gyro_getADC(void);
int Baro_update(void);
bool initBaro(baro_t * baro);
bool initSonar();
int32_t pollSonar();

// Output
void mixerInit(void);
void mixerResetMotors(void);
void writeMotors(void);
void writeAllMotors(int16_t mc);
void mixTable(void);

// Serial
void serialInit(uint32_t baudrate);
void serialCom(void);

// rxmsp
void mspInit(rcReadRawDataPtr *callback);
bool mspFrameComplete(void);
void mspFrameRecieve(void);

// buzzer
void systemBeep(bool onoff);
