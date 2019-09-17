
/*
   Arduino sketch to test motors on Ladybug Flight Controller

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

static uint8_t MOTOR_PIN = 13; // Motor 1: right rear
//static uint8_t MOTOR_PIN = A2; // Motor 2: right front
//static uint8_t MOTOR_PIN = 3;  // Motor 3: left rear
//static uint8_t MOTOR_PIN = 11; // Motor 4: left front

static uint8_t val;
static uint8_t inc;

void setup(void)
{
    // Initialize the motor
    analogWriteFrequency(MOTOR_PIN, 10000);  
    analogWrite(MOTOR_PIN, 0);  

    // Start with motor off, increasing
    val = 0;
    inc = +1;
}

void loop(void)
{
    analogWrite(MOTOR_PIN, val);

    val += inc;

    // stop halfway
    if (val == 128) {
        inc = -1;
    }

    if (val == 0) {
        inc = +1;
    }

    delay(10);
}
