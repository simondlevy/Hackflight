/*
   Simple test sketch for sending DSHOT600 ESC commands from Teensy4.0

   Copyright (c) 2020 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "DSHOT.h"

static int8_t   dir;

static const uint16_t MINVAL = 48;
static const uint16_t MAXVAL = 2047;

static uint8_t telem;
static uint16_t val;

void setup(void)
{
    Serial.begin(115200);

    DSHOT_init(1);

    telem = 0;
    val = 0;

    dir = +1;
}

void loop(void)
{
    static int count;

    if (count++<500) {
        DSHOT_send(&val, &telem);
    }

    /*
    val[0] += dir;

    if (val[0] == MAXVAL) {
        dir = -1;
    }

    if (val[0] == MINVAL) {
        dir = +1;
    }
    */
    delay(10);
}
