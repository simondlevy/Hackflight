/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

#define _EXTERN
#include "copilot.h"

#define NONEXTERN
#include "stream_receiver.h"


void stream_updateReceiver(void)
{
    stream_receiverAux1 = -1;
    stream_receiverThrottle = -1;
    stream_receiverLostSignal = false;
}
