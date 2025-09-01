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

static const std::vector<uint8_t> MOTOR_PINS = {PA1, PB11};

static const std::vector<uint8_t> POWER_SWITCH_PINS = {PA0, PB12};

static auto motors = OneShot125(MOTOR_PINS);

static void enableMotor(const uint8_t id)
{
    pinMode(POWER_SWITCH_PINS[id], OUTPUT);
    digitalWrite(POWER_SWITCH_PINS[id], HIGH);
}

void setup() 
{
    Serial.begin(115200);

    inputInit();

    enableMotor(0);
    enableMotor(1);

    motors.arm(); 
}

void loop() 
{
    auto pulseWidth = (uint8_t)(125 * (inputGet() + 1));

    motors.set(0, pulseWidth);
    motors.set(1, pulseWidth);

    motors.run();
}
