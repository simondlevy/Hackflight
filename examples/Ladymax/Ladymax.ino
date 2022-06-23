/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Wire.h>

#include <hackflight_full.h>
#include <stm32_clock.h>

static hackflight_t _hf;

void setup(void)
{
    Wire.begin();
    delay(100);

    void * motorDevice = NULL;

    // Always use Serial1 for receiver, no no need to specify
    hackflightInitFull(&_hf, motorDevice, SERIAL_PORT_NONE, 5, A4);

    stm32_startCycleCounter();
}

void loop(void)
{
    hackflightStep(&_hf);
}
