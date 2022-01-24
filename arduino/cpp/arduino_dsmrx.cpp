/*
   Arduino support for Spketrum DSMX receiver

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <DSMRX.h>

#define _EXTERN
#include "hackflight.h"

#include "arduino_debugger.hpp"

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

void dsmrxGetInt(void)
{
    uint16_t rawvals[8];

    rx.getChannelValues(rawvals, 8);

    /*
    int_receiverThrottle = rawvals[0];
    int_receiverRoll     = rawvals[1];
    int_receiverPitch    = rawvals[2];
    int_receiverYaw      = rawvals[3];
    int_receiverAux1     = rawvals[6];
    int_receiverAux2     = rawvals[4];
    */
}
void dsmrxGet(void)
{
    float frawvals[8];
    rx.getChannelValues(frawvals, 8);
    receiverThrottle = frawvals[0];
    receiverRoll     = frawvals[1];
    receiverPitch    = frawvals[2];
    receiverYaw      = frawvals[3];
    receiverAux1     = frawvals[6];
    receiverAux2     = frawvals[4];

    uint16_t irawvals[8];
    rx.getChannelValues(irawvals, 8);
    /*
    int_receiverThrottle = irawvals[0];
    int_receiverRoll     = irawvals[1];
    int_receiverPitch    = irawvals[2];
    int_receiverYaw      = irawvals[3];
    int_receiverAux1     = irawvals[6];
    int_receiverAux2     = irawvals[4];
    */
}

void ignore(uint16_t t, uint16_t r, uint16_t p, uint16_t y, uint16_t a1, uint16_t a2)
{
}
