/*
   Arduino time support

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <Arduino.h>

#define _EXTERN
#include "hackflight.h"

void timeUpdate(void)
{
    usec = micros();
}
