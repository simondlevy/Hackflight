#pragma once

#ifdef _EXTERN
#define EXTERN extern
#else
#define EXTERN
#endif

#include <stdint.h>
#include <stdbool.h>

EXTERN float stream_receiverThrottle;
EXTERN float stream_time;
EXTERN float stream_agl;
EXTERN uint32_t stream_micros;
EXTERN float stream_receiverRoll;
EXTERN float stream_receiverPitch;
EXTERN float stream_receiverYaw;
void stream_getReceiverDemands(void);
void stream_setMotors(float stream_setMotors_arg0, float stream_setMotors_arg1, float stream_setMotors_arg2, float stream_setMotors_arg3);
void stream_setPose(float stream_setPose_arg0, float stream_setPose_arg1, float stream_setPose_arg2, float stream_setPose_arg3, float stream_setPose_arg4, float stream_setPose_arg5);
void step(void);
