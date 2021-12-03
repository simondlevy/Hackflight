/*
 * Spektrum DSMX reciever support for ESP32
 *
 * Copyright (c) 2021 Simon D. Levy
 *
 * MIT license
 */

#include <Arduino.h>
#include <DSMRX.h>

#define _EXTERN
#include "hackflight.h"

DSM2048 rx;
static bool rxReady;

static void coreTask(void * params)
{
    while (true) {
      
        if (Serial1.available()) {
           rx.handleSerialEvent(Serial1.read(), micros()); 
           rxReady = true;
        }

        delay(1);
    } 
}

void stream_startDsmrx(uint8_t rxpin, uint8_t txpin)
{
    Serial1.begin(115000, SERIAL_8N1, rxpin, txpin);
    TaskHandle_t task;
    xTaskCreatePinnedToCore(coreTask, "Task", 10000, NULL, 1, &task, 0); 
}

void stream_updateDsmrx(void)
{
    stream_receiverTimedOut = rx.timedOut(micros());
    stream_receiverGotNewFrame = rx.gotNewFrame();
}

void stream_getDsmrx(void)
{
    float rawvals[8];

    rx.getChannelValues(rawvals, 8);

    stream_receiverThrottle = rawvals[0];
    stream_receiverRoll     = rawvals[1];
    stream_receiverPitch    = rawvals[2];
    stream_receiverYaw      = rawvals[3];
    stream_receiverAux1     = rawvals[6];
    stream_receiverAux2     = rawvals[4];
}

void stream_ignore(bool timedOut, float aux2)
{
}
