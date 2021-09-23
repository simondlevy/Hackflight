/*
   Arduino I^2C support for Haskell Copilot

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <Wire.h>

#define _EXTERN
#include "copilot.h"

void copilot_startWire(void)
{
    Wire.begin();
}
