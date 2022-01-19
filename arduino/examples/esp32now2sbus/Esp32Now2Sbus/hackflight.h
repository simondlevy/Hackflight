#pragma once

#ifdef _EXTERN
#define EXTERN extern
#else
#define EXTERN
#endif

#include <stdint.h>
#include <stdbool.h>

void serialStart(void);
void esp32nowStart(void);
void esp32nowAddPeer(uint8_t esp32nowAddPeer_arg0, uint8_t esp32nowAddPeer_arg1, uint8_t esp32nowAddPeer_arg2, uint8_t esp32nowAddPeer_arg3, uint8_t esp32nowAddPeer_arg4, uint8_t esp32nowAddPeer_arg5);
void esp32nowRegisterReceiveCallback(void);
void esp32nowRead(void);
void esp32nowDebug(void);
void delayMsec(uint32_t delayMsec_arg0);
void step(void);
