/*
   Test TinyPICO running DSHOT600 ESC protocol


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

#include "hackflight.hpp"
#include "motors/esp32dshot600.hpp"

static const uint8_t PINS[1] = {27};

hf::NewEsp32DShot600 motors = hf::NewEsp32DShot600(PINS, 1);

static float val;
static int8_t dir;

void setup(void)
{
    motors.init();

    delay(1000);

    val = 0;
    dir = +1;
}

void loop(void)
{
    motors.write(0, val);

    val += dir * .001;

    if (val >= .5) dir = -1;
    if (val <=  0) dir = +1;

    delay(10);
}
