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

#define RX_SERIAL Serial5

// Un-comment on of these:
//#include "input_dsmx.hpp"
#include "input_sbus.hpp"

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

    motors.set(0, pulseWidth);
    motors.set(1, pulseWidth);
    motors.set(2, pulseWidth);
    motors.set(3, pulseWidth);

    motors.run();
}
