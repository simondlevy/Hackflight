/*
   Arduino time support

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <Arduino.h>

#define _EXTERN
#include "copilot.h"

void stream_updateTime(void)
{
    stream_micros = micros();
}
