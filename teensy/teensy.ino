/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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


#define _MAIN

#include <hackflight.hpp>
#include <mixers/crazyflie.hpp>

static Hackflight hackflight;

static void wait(const float freq)
{
    delay(1000/freq);
}

void setup() 
{
    pinMode(16, INPUT_PULLUP);
    pinMode(17, INPUT_PULLUP);
    pinMode(18, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);

    hackflight.init(15, false, &Serial1); //&Wire1, &SPI, SS);
}

void loop() 
{
    wait(Hackflight::LOOP1_TASK_FREQ);

    hackflight.loop1(Mixer::rotorCount, Mixer::mix);
}
