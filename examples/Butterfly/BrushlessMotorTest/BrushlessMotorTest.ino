/*
   Arduino sketch to test motors on Butterfly flight Controller

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

#include <Arduino.h>

static uint8_t MOTOR_PIN = 11;

static uint16_t val;
static int16_t inc;

// Min, max PWM values
const uint16_t PWM_MIN = 1000;
const uint16_t PWM_MAX = 2000;


void setup(void)
{
    // Initialize the motor
    pinMode(MOTOR_PIN, OUTPUT);
    analogWrite(MOTOR_PIN, PWM_MIN>>3);

    // Start with motor off, increasing
    val = PWM_MIN;
    inc = +1;

    delay(1000);
}

void loop(void)
{
    analogWrite(MOTOR_PIN, val >> 3);

    val += inc;

    // stop halfway
    if (val >= (PWM_MIN+PWM_MAX)/2) {
        inc = -1;
    }

    if (val <= PWM_MIN) {
        inc = +1;
    }

    delay(10);
}
