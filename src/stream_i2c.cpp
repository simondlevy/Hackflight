/*
   Hackflight stream-based I^2C support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <Wire.h>

void stream_startJ2C(void) // sic
{
    Wire.begin();
    delay(100);
}
