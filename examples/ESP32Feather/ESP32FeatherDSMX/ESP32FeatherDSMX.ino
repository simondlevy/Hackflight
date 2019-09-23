/*
   Hackflight sketch for ESP32 Feather with DSMX receiver

   Copyright (c) 2019 Simon D. Levy

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

#include <Arduino.h>

#include "hackflight.hpp"
#include "boards/arduino/esp32feather.hpp"
#include "receivers/mock.hpp"
#include "receivers/arduino/dsmx.hpp"
#include "pidcontrollers/rate.hpp"
#include "mixers/quadxcf.hpp"

const uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

hf::Hackflight h;

hf::DSMX_Receiver rc = hf::DSMX_Receiver(CHANNEL_MAP, &Serial2);

hf::MixerQuadXCF mixer;

hf::RatePid ratePid = hf::RatePid(0, 0, 0, 0, 0);

static void coreTask(void * params)
{
    while (true) {

        if (Serial2.available()) {
           rx.handleSerialEvent(micros()); // rx is included by dsmx.hpp
        }

        delay(1);
    }
}

void setup(void)
{
     // Initialize Hackflight firmware
     h.init(new hf::ESP32FeatherBoard(), &rc, &mixer, &ratePid);

    // Start the timer task for the receiver
    TaskHandle_t task;
    xTaskCreatePinnedToCore(coreTask, "Task", 10000, NULL, 1, &task, 0);
}

void loop(void)
{
    h.update();

    delay(10);
}
