/*
   Arduino support for Haskell Copilot

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <Wire.h>

#define _EXTERN
#include "copilot_extra.h"

// Serial comms ---------------------------------------------------------------

void copilot_startSerial(void)
{
    Serial.begin(115200);
}

void copilot_serialWrite(uint8_t b)
{
    Serial.write(b);
}

void copilot_updateSerial(void)
{
    copilot_serialAvailable = Serial.available();

    if (copilot_serialAvailable) {
        copilot_serialByte = Serial.read();
    }
}

static uint8_t serialInputBuffer[128] = {};

float copilot_getFloatFromSerialInput(uint8_t offset)
{
    float value = 0;
    memcpy(&value,  &serialInputBuffer[offset], sizeof(float));
    return value;
}

void copilot_collectSerialInput(uint8_t index, uint8_t value)
{
   serialInputBuffer[index] = value;
}

void copilot_convertFloat(float value)
{
    memcpy(&copilot_32bits, &value, 4);
}

// LED---------------------------------------------------------------

void copilot_startLed(uint8_t pin)
{
    pinMode(pin, OUTPUT);
}

void copilot_setLed(uint8_t pin, bool on)
{
    digitalWrite(pin, on);
}

// Clock ---------------------------------------------------------------

void copilot_updateClock(void)
{
    copilot_time_sec = micros() / 1.e6f;
}

// I^2C  ---------------------------------------------------------------
void copilot_startWire(void)
{
    Wire.begin();
}
