/**
 *
 * Copyright 2025 Simon D. Levy
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

#include "bootloader.hpp"

#include <hackflight.hpp>
#include <mixers/crazyflie.hpp>

static Hackflight hackflight;

void setup()
{
    static HardwareSerial uart = HardwareSerial(PC7, PC6);

    hackflight.init1(PC14, true, &uart);
}

void loop()
{  
    delay(1);

    hackflight.loop1(Mixer::rotorCount, Mixer::mix);

    if (Serial.available() && Serial.read() == 'R') {
        Bootloader::jump();
    }
}
