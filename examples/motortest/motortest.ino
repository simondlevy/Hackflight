/*
   Test ESCs.  Make sure to run Calibrate sketch first

   This file is part of Teensy-OneShot125.

   Teensy-OneShot125 is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Teensy-OneShot125 is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Teensy-OneShot125. If not, see <https://www.gnu.org/licenses/>.
 */

#include <oneshot125.hpp>
#include <vector>

static const std::vector<uint8_t> PINS = {2, 23, 14, 9};

static float MAX = 0.5;
static float INC = 1e-5;

static auto motors = OneShot125(PINS);

static float val;
static float dir;

void setup() 
{
    Serial.begin(115200);

    motors.arm(); 

    val = 0;
    dir = +1;

}

static float inputGet()
{
    val += INC * dir;

    if (val >= MAX) {
        dir = -1;
    }

    if (val <= 0) {
        dir = +1;
    }
    
    return val;
}

void loop() 
{
    const float input = inputGet();

    auto pulseWidth = (uint8_t)(125 * (input + 1));

    motors.set(0, pulseWidth);
    motors.set(1, pulseWidth);
    motors.set(2, pulseWidth);
    motors.set(3, pulseWidth);

    motors.run();
}
