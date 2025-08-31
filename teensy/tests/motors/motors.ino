/*
   Test ESCs -- remove propellers first!

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <oneshot125.hpp>
#include <input_keyboard.hpp>

#include <vector>

// XXX not sure why we have to do this
static const uint8_t M1_OFFSET = 22;
static const uint8_t M2_OFFSET = 0;
static const uint8_t M3_OFFSET = 22;
static const uint8_t M4_OFFSET = 0;

static const std::vector<uint8_t> PINS = {2, 23, 14, 9};

static auto motors = OneShot125(PINS);

void setup() 
{
    Serial.begin(115200);

    inputInit();

    motors.arm(); 
}

void loop() 
{
    auto pulseWidth = (uint8_t)(125 * (inputGet() + 1));


    motors.set(0, pulseWidth + M1_OFFSET);
    motors.set(1, pulseWidth + M2_OFFSET);
    motors.set(2, pulseWidth + M3_OFFSET);
    motors.set(3, pulseWidth + M4_OFFSET);

    motors.run();
}
