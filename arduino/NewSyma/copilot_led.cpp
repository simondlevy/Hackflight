/*
   Arduino LED support for Haskell Copilot

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>


#define _EXTERN
#include "copilot.h"

static uint8_t _pin;

void copilot_startLed(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    _pin = pin;
}

void copilot_setLed(bool on)
{
    digitalWrite(_pin, on);
}
