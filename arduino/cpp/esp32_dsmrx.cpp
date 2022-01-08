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

void dsmrxStart(uint8_t rxpin, uint8_t txpin)
{
    Serial1.begin(115000, SERIAL_8N1, rxpin, txpin);
    TaskHandle_t task;
    xTaskCreatePinnedToCore(coreTask, "Task", 10000, NULL, 1, &task, 0); 
}

void dsmrxUpdate(void)
{
    // receiverTimedOut = rx.timedOut(micros());
    receiverGotNewFrame = rx.gotNewFrame();
}

void dsmrxGet(void)
{
    float rawvals[8];

    rx.getChannelValues(rawvals, 8);

    /*
    receiverThrottle = rawvals[0];
    receiverRoll     = rawvals[1];
    receiverPitch    = rawvals[2];
    receiverYaw      = rawvals[3];
    receiverAux1     = rawvals[6];
    receiverAux2     = rawvals[4];
    */
}
