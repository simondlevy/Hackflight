/*
   Arduino support for Spketrum DSMX receiver

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <DSMRX.h>

#define _EXTERN
#include "hackflight.h"

static DSM2048 rx;

static bool rxReady;

void serialEvent1(void)
{
    rxReady = true;

    while (Serial1.available()) {

        rx.handleSerialEvent(Serial1.read(), micros());
    }
}

void stream_startDsmrx(void)
{
    Serial1.begin(115200);
    stream_receiverLostSignal = false;
}

void stream_updateDsmrx(void)
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
