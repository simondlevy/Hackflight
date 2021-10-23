/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

#define _EXTERN
#include "copilot.h"

#define NONEXTERN
#include "stream_receiver.h"

float stream_receiverRoll;
float stream_receiverPitch;
float stream_receiverYaw;

void stream_updateReceiver(void)
{
    stream_receiverAux1 = -1;
    stream_receiverThrottle = -1;
    stream_receiverLostSignal = false;

    stream_receiverRoll = 0.2;
    stream_receiverPitch = -0.1; 
    stream_receiverYaw = 0;
}
