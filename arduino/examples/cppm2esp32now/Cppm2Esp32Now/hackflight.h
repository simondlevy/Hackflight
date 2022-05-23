#pragma once

#ifdef _EXTERN
#define EXTERN extern
#else
#define EXTERN
#endif

#include <stdint.h>
#include <stdbool.h>

EXTERN bool receiverGotNewFrame;
EXTERN uint16_t receiverChan1;
EXTERN uint16_t receiverChan2;
EXTERN uint16_t receiverChan3;
EXTERN uint16_t receiverChan4;
EXTERN uint16_t receiverChan5;
EXTERN uint16_t receiverChan6;
void serialStart(void);
void cppmStart(uint8_t cppmStart_arg0, uint8_t cppmStart_arg1);
void esp32nowStart(void);
void esp32nowAddPeer(uint8_t esp32nowAddPeer_arg0, uint8_t esp32nowAddPeer_arg1, uint8_t esp32nowAddPeer_arg2, uint8_t esp32nowAddPeer_arg3, uint8_t esp32nowAddPeer_arg4, uint8_t esp32nowAddPeer_arg5);
void cppmUpdate(void);
void cppmGet(void);
void esp32nowPrepareToSend(uint8_t esp32nowPrepareToSend_arg0, uint8_t esp32nowPrepareToSend_arg1, uint8_t esp32nowPrepareToSend_arg2, uint8_t esp32nowPrepareToSend_arg3, uint8_t esp32nowPrepareToSend_arg4, uint8_t esp32nowPrepareToSend_arg5);
void commsSend(uint8_t commsSend_arg0, uint8_t commsSend_arg1, uint8_t commsSend_arg2, float commsSend_arg3, float commsSend_arg4, float commsSend_arg5, float commsSend_arg6, float commsSend_arg7, float commsSend_arg8);
void step(void);
