#pragma once

#ifdef _EXTERN
#define EXTERN extern
#else
#define EXTERN
#endif

#include <stdint.h>
#include <stdbool.h>

EXTERN float stream_receiverThrottle;
EXTERN float stream_stateDz;
EXTERN float stream_statePsi;
EXTERN float stream_stateDy;
EXTERN double stream_time;
EXTERN float stream_stateDx;
EXTERN float stream_stateDphi;
EXTERN uint32_t stream_micros;
EXTERN float stream_receiverRoll;
EXTERN float stream_statePhi;
EXTERN float stream_stateDtheta;
EXTERN float stream_receiverPitch;
EXTERN float stream_stateTheta;
EXTERN float stream_stateDpsi;
EXTERN float stream_receiverYaw;
void stream_getReceiverDemands(void);
void stream_writeMotors(float stream_writeMotors_arg0, float stream_writeMotors_arg1, float stream_writeMotors_arg2, float stream_writeMotors_arg3);
void step(void);
