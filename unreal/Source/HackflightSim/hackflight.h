#pragma once

#ifdef _EXTERN
#define EXTERN extern
#else
#define EXTERN
#endif

#include <stdint.h>
#include <stdbool.h>
EXTERN float stream_receiverThrottle;
EXTERN float stream_altimeterZ;
EXTERN float stream_time;
EXTERN uint32_t stream_micros;
EXTERN float stream_receiverRoll;
EXTERN bool stream_imuGotQuaternion;
EXTERN float stream_imuQuaternionX;
EXTERN float stream_imuQuaternionY;
EXTERN float stream_imuQuaternionW;
EXTERN float stream_imuQuaternionZ;
EXTERN float stream_flowX;
EXTERN float stream_flowY;
EXTERN bool stream_imuGotGyrometer;
EXTERN float stream_imuGyrometerX;
EXTERN float stream_receiverPitch;
EXTERN float stream_imuGyrometerY;
EXTERN float stream_receiverYaw;
EXTERN float stream_imuGyrometerZ;
void stream_getReceiverDemands(void);
void stream_getGyrometer(void);
void stream_getQuaternion(void);
void stream_getOpticalFlow(void);
void stream_writeMotors(float stream_writeMotors_arg0, float stream_writeMotors_arg1, float stream_writeMotors_arg2, float stream_writeMotors_arg3);
void step(void);
