
/*
   BrushlessMotorTest.ino : Arduino sketch to test motors on Butterfly Flight Controller

   DID YOU REMEMOVE THE PROPELLERS FIRST?

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

#include <Servo.h>

static uint8_t MOTOR_PIN = 3;  

static uint16_t BASELINE = 990;
static uint16_t MAXVAL   = 1500;

static uint16_t val;
static int8_t   inc;

Servo esc;

void setup(void)
{
    // Connect to the ESC
    esc.attach(MOTOR_PIN);

    // Send the baseline value to the ESC
    esc.writeMicroseconds(BASELINE);

    // Start with motor off, increasing
    val = BASELINE;
    inc = +1;

    // Wait a spell
    delay(1000);
}

void loop(void)
{
    // Send the current value to the ESC
    esc.writeMicroseconds(val);

    // Increement or decrement value
    val += inc;

    // At max, switch to decrement
    if (val == MAXVAL) {
        inc = -1;
    }

    // At min, switch to increment
    if (val == BASELINE) {
        inc = +1;
    }

    // Wait a bit between updates
    delay(10);
}
