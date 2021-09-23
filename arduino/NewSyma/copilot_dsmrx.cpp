/*
   Support for DSMX receiver in Haskell Copilot

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

/*

#define _EXTERN
#include "copilot.h"

#include <Arduino.h>
#include <DSMRX.h>

static DSM2048 _rx;
static uint32_t _timeouts;

void serialEvent1(void)
{
    while (Serial1.available()) {
        _rx.handleSerialEvent(Serial1.read(), micros());
    }
}

void copilot_startDsmrx(void)
{
    Serial1.begin(115200);

    copilot_receiverLostSignal = false;
}

void copilot_updateDsmrx(void)
{
    if (_rx.timedOut(micros())) {
        _timeouts++;
        if (_timeouts > 200) {
            copilot_receiverLostSignal = true;
        }
    }

    else if (_rx.gotNewFrame()) {

        float values[8] = {};

        _rx.getChannelValues(values);

        copilot_receiverThrottle = values[0];
        copilot_receiverRoll = values[1];
        copilot_receiverPitch = values[2];
        copilot_receiverYaw = values[3];
        copilot_receiverAux1 = values[6];
    }
}
*/
