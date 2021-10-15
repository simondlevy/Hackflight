/*
   Haskell Copilot receiver support
 */

#pragma once

#ifdef NONEXTERN
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN bool stream_receiverLostSignal;
EXTERN float stream_receiverAux2;

void stream_startReceiver(void);
void stream_updateReceiver(void);
