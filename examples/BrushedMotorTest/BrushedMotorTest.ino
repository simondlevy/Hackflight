/*
   DID YOU REMEMOVE THE ROTORS FIRST?

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

static uint8_t PIN = 13;
static uint8_t MAXVAL = 100;

static int8_t val;
static int8_t dir;

void setup(void)
{
    // Initialize the motor
    analogWriteFrequency(PIN, 10000);
    analogWrite(PIN, 0);

    // Start with motor off, increasing
    val = 0;
    dir = +1;

    delay(1000);
}

void loop(void)
{
    analogWrite(PIN, val);

    val += dir;

    // stop halfway
    if (val == MAXVAL) {
        dir = -1;
    }

    if (val == 0) {
        dir = +1;
    }

    delay(10);
}
