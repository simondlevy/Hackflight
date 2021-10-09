/*
   Haskell Copilot support for Spketrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/SpektrumDSM 

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <DSMRX.h>

#define NONEXTERN
#include "stream_receiver.h"

static DSM2048 rx;

static bool rxReady;

void serialEvent1(void)
{
    rxReady = true;

    while (Serial1.available()) {

        rx.handleSerialEvent(Serial1.read(), micros());
    }
}


void stream_startReceiver(void)
{
    Serial1.begin(115200);
    stream_receiverLostSignal = false;
}

void stream_updateReceiver(void)
{
    if (rx.timedOut(micros())) {
        if (rxReady) {
            stream_receiverLostSignal = true;
        }
    }

    else if (rx.gotNewFrame()) {

        float rawvals[8];

        rx.getChannelValues(rawvals, 8);

        stream_receiverThrottle = rawvals[0];
        stream_receiverRoll     = rawvals[1];
        stream_receiverPitch    = rawvals[2];
        stream_receiverYaw      = rawvals[3];
        stream_receiverAux1     = rawvals[6];
        stream_receiverAux2     = rawvals[4];
    }
}
