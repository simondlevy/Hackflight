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


#define _MAIN

#include <TeensyThreads.h>

#include <hackflight.hpp>
#include <mixers/crazyflie.hpp>

static Hackflight hackflight;

static void loop2_thread()
{
    while(true) {

        hackflight.loop2();
        threads.delay(20);
        threads.yield();
    }
}

void setup() 
{
    Serial.begin(115200);

    SPI.begin();

    pinMode(16, INPUT_PULLUP);
    pinMode(17, INPUT_PULLUP);
    pinMode(18, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);

    hackflight.init(15, false, &Serial1, &Wire1, &SPI, SS);

    threads.addThread(loop2_thread);
}

void loop() 
{
    hackflight.loop1(Mixer::rotorCount, Mixer::mix);
    delayMicroseconds(1);
}
