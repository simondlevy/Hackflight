/*
   Arduino time support

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <Arduino.h>

#define _EXTERN
#include "hackflight.h"

void stream_updateTime(void)
{
    stream_micros = micros();

    // stream_time = stream_micros / 1e6;
}
