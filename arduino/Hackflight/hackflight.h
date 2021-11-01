#pragma once

#ifdef _EXTERN
#define EXTERN extern
#else
#define EXTERN
#endif

#include <stdint.h>
#include <stdbool.h>
EXTERN bool stream_imuGotQuaternion;
EXTERN float stream_imuQuaternionW;
EXTERN float stream_imuQuaternionX;
EXTERN float stream_imuQuaternionY;
EXTERN float stream_imuQuaternionZ;
EXTERN float stream_receiverAux1;
EXTERN float stream_receiverThrottle;
EXTERN bool stream_receiverLostSignal;
EXTERN uint8_t stream_serialByte;
EXTERN bool stream_imuGotGyrometer;
EXTERN float stream_imuGyrometerX;
EXTERN uint32_t stream_micros;
EXTERN float stream_receiverRoll;
EXTERN float stream_imuGyrometerY;
EXTERN float stream_receiverPitch;
EXTERN float stream_imuGyrometerZ;
EXTERN float stream_receiverYaw;
EXTERN bool stream_serialAvailable;
EXTERN float stream_receiverAux2;
void stream_startSerial(void);
void stream_startI2C(void);
void stream_startDsmrx(void);
void stream_startUsfs(void);
void stream_startBrushedMotors(uint8_t stream_startBrushedMotors_arg0, uint8_t stream_startBrushedMotors_arg1, uint8_t stream_startBrushedMotors_arg2, uint8_t stream_startBrushedMotors_arg3);
void stream_startLed(uint8_t stream_startLed_arg0);
void stream_updateUsfs(void);
void stream_updateDsmrx(void);
void stream_updateTime(void);
void stream_writeLed(uint8_t stream_writeLed_arg0, bool stream_writeLed_arg1);
void stream_serialUpdate(void);
void stream_serialRead(void);
void stream_serialSend(uint8_t stream_serialSend_arg0, uint8_t stream_serialSend_arg1, uint8_t stream_serialSend_arg2, uint8_t stream_serialSend_arg3, uint8_t stream_serialSend_arg4, uint8_t stream_serialSend_arg5, uint8_t stream_serialSend_arg6, float stream_serialSend_arg7, float stream_serialSend_arg8, float stream_serialSend_arg9, float stream_serialSend_arg10, float stream_serialSend_arg11, float stream_serialSend_arg12);
void stream_writeBrushedMotors(uint8_t stream_writeBrushedMotors_arg0, uint8_t stream_writeBrushedMotors_arg1, uint8_t stream_writeBrushedMotors_arg2, uint8_t stream_writeBrushedMotors_arg3, float stream_writeBrushedMotors_arg4, float stream_writeBrushedMotors_arg5, float stream_writeBrushedMotors_arg6, float stream_writeBrushedMotors_arg7);
void step(void);
