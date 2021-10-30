/*
   Arduino LED support

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <Arduino.h>

void stream_startLed(const uint8_t pin)
{
    pinMode(pin, OUTPUT);
}

void stream_writeLed(const uint8_t pin, bool value)
{
    digitalWrite(pin, value);
}
