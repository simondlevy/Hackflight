/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <oneshot125.hpp>
#include <input_keyboard.hpp>

static const std::vector<uint8_t> MOTOR_PINS = {PB5, PB4, PB6, PB7};

static auto motors = OneShot125(MOTOR_PINS);

void setup()
{
    Serial.begin(115200);

    motors.arm(); }

void loop()
{
    auto pulseWidth = (uint8_t)(125 * (inputGet() + 1));

    motors.set(0, pulseWidth);
    motors.set(1, pulseWidth);
    motors.set(2, pulseWidth);
    motors.set(3, pulseWidth);


    motors.run();
}

