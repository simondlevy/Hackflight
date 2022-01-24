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

#ifdef ESP32
static void coreTask(void * params)
{
    while (true) {
      
        if (Serial1.available()) {
           rx.handleSerialEvent(Serial1.read(), micros()); 
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
#else
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
#endif

void dsmrxUpdate(void)
{
    // receiverTimedOut = rx.timedOut(micros());
    receiverGotNewFrame = rx.gotNewFrame();
}

void dsmrxGet(void)
{
    uint16_t rawvals[8];

    rx.getChannelValues(rawvals, 8);

    receiverThrottle = rawvals[0];
    receiverRoll     = rawvals[1];
    receiverPitch    = rawvals[2];
    receiverYaw      = rawvals[3];
    receiverAux1     = rawvals[6];
    receiverAux2     = rawvals[4];
}

void dump(uint16_t t,
          uint16_t r, 
          uint16_t p, 
          uint16_t y, 
          uint16_t a1, 
          uint16_t a2)
{
    Debugger::printf("%04d %04d %04d %04d %04d %04d\n", t, r, p, y, a1, a2);
}

