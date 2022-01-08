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

void serialEvent1(void)
{
    receiverReady = true;

    while (Serial1.available()) {

        rx.handleSerialEvent(Serial1.read(), micros());
    }
}

void dsmrxStart(void)
{
    Serial1.begin(115200);
}

void dsmrxUpdate(void)
{
    receiverTimedOut = rx.timedOut(micros());
    receiverGotNewFrame = rx.gotNewFrame();
}

void dsmrxGet(void)
{
    float rawvals[8];

    rx.getChannelValues(rawvals, 8);

    receiverThrottle = rawvals[0];
    receiverRoll     = rawvals[1];
    receiverPitch    = rawvals[2];
    receiverYaw      = rawvals[3];
    receiverAux1     = rawvals[6];
    receiverAux2     = rawvals[4];
}
