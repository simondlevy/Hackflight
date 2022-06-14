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

#include <hackflight.h>

static hackflight_t _hf;

void setup(void)
{
    Wire.begin();
    delay(100);

    // Always use Serial1 for receiver, no no need to specify
    hackflightFullInit(&_hf, SERIAL_PORT_NONE);

    // Setup system clock ------------------------------------

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    __O uint32_t *DWTLAR = (uint32_t *)(DWT_BASE + 0x0FB0);
    *(DWTLAR) = 0xC5ACCE55;
    
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void loop(void)
{
    hackflightStep(&_hf);
}
