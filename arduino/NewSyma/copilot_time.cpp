/*
   Arduino time support for Haskell Copilot

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

#define _EXTERN
#include "copilot.h"

void copilot_updateTime(void)
{
    copilot_time_msec = millis();
    // copilot_time_sec = micros() / 1.e6f;
}
