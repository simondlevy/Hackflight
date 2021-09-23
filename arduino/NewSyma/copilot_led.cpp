/*
   Arduino LED support for Haskell Copilot

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>


#define _EXTERN
#include "copilot.h"


void copilot_startLed(uint8_t pin)
{
    pinMode(pin, OUTPUT);
}

void copilot_setLed(uint8_t pin, bool on)
{
    digitalWrite(pin, on);
}
