#pragma once

#ifdef _EXTERN
#define EXTERN extern
#else
#define EXTERN
#endif

#include <stdint.h>
EXTERN bool copilot_receiverLostSignal;
EXTERN float copilot_receiverAux1;
EXTERN float copilot_receiverThrottle;
EXTERN float copilot_quaternionW;
EXTERN float copilot_quaternionX;
EXTERN float copilot_quaternionY;
EXTERN float copilot_quaternionZ;
EXTERN float copilot_gyrometerX;
EXTERN float copilot_receiverRoll;
EXTERN float copilot_gyrometerY;
EXTERN float copilot_receiverPitch;
EXTERN float copilot_receiverYaw;
EXTERN float copilot_gyrometerZ;
EXTERN uint32_t copilot_time_msec;
void copilot_writeMotor(uint8_t copilot_writeMotor_arg0, float copilot_writeMotor_arg1);
void copilot_writeMotor(uint8_t copilot_writeMotor_arg0, float copilot_writeMotor_arg1);
void copilot_writeMotor(uint8_t copilot_writeMotor_arg0, float copilot_writeMotor_arg1);
void copilot_writeMotor(uint8_t copilot_writeMotor_arg0, float copilot_writeMotor_arg1);
void copilot_setLed(bool copilot_setLed_arg0);
void copilot_serialWrite(uint8_t copilot_serialWrite_arg0);
void step(void);
