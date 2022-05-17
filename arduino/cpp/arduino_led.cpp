/*
   Arduino LED support

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <Arduino.h>

void ledStart(const uint8_t pin)
{
    pinMode(pin, OUTPUT);
}

void ledWrite(const uint8_t pin, bool value)
{
    digitalWrite(pin, value);
}
