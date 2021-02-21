/*
   Arduino sketch to test brushless motor with standard ESC

   DID YOU REMEMOVE THE PROPELLERS FIRST?

   Copyright (c) 2018 Simon D. Levy

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
#include "motors/standard.hpp"

static uint8_t MOTOR_PIN[1] = {15};

static float  val;
static int8_t dir;

hf::StandardMotor motors = hf::StandardMotor(MOTOR_PIN, 1);

void setup(void)
{
    // Initialize the motor
    motors.init();

    // Start with motor off, increasing
    val = 0;
    dir = +1;

    delay(1000);
}

void loop(void)
{
    motors.write(0, val);

    val += dir * .001;

    // stop halfway
    if (val >= 0.5) {
        dir = -1;
    }

    if (val <= 0) {
        dir = +1;
    }

    delay(10);
}
