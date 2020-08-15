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

void setup(void)
{
    Serial.begin(115200);

    DSHOT_init(1);
}

void loop(void)
{
    static int count;
    static uint16_t runval;
    uint8_t telem = 0;

    if (count < 500) {

        uint16_t stopval = 0;
        DSHOT_send(&stopval, &telem);
        runval = 1049;
    }

    else {

        for (uint8_t k=0; k<25; ++k) {
            DSHOT_send(&runval, &telem);
            delay(10);
        }

        runval += 1;
    }

    count++;
}
