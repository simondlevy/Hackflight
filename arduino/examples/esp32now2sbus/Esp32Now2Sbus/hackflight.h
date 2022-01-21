#pragma once

#ifdef _EXTERN
#define EXTERN extern
#else
#define EXTERN
#endif

#include <stdint.h>
#include <stdbool.h>

EXTERN uint8_t esp32nowByte;
void serialStart(void);
void esp32nowStart(void);
void esp32nowAddPeer(uint8_t esp32nowAddPeer_arg0, uint8_t esp32nowAddPeer_arg1, uint8_t esp32nowAddPeer_arg2, uint8_t esp32nowAddPeer_arg3, uint8_t esp32nowAddPeer_arg4, uint8_t esp32nowAddPeer_arg5);
void esp32nowRegisterReceiveCallback(void);
void esp32nowRead(void);
void delayUsec(uint32_t delayUsec_arg0);
void esp32nowDebug(float esp32nowDebug_arg0, float esp32nowDebug_arg1, float esp32nowDebug_arg2, float esp32nowDebug_arg3, float esp32nowDebug_arg4, float esp32nowDebug_arg5);
void step(void);
