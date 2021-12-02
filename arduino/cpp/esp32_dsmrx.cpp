/*
 * Spektrum DSMX reciever support for ESP32
 *
 * Copyright (c) 2021 Simon D. Levy
 *
 * MIT license
 */

#include <DSMRX.h>

static const uint8_t DSMX_RX_PIN    = 4;
static const uint8_t DSMX_TX_PIN    = 14; // unused
static const uint8_t DSMX_CHANNELS  = 8;

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

void stream_startDsmrx(void)
{
    Serial1.begin(115000, SERIAL_8N1, DSMX_RX_PIN, DSMX_TX_PIN);
    TaskHandle_t task;
    xTaskCreatePinnedToCore(coreTask, "Task", 10000, NULL, 1, &task, 0); 
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

