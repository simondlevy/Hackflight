/*
   Hackflight sketch for TinyPICO with DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/EM7180
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

       https://github.com/plerup/espsoftwareserial


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

#include <SoftwareSerial.h>

#include "hackflight.hpp"
#include "boards/arduino/tinypico.hpp"
#include "receivers/arduino/dsmx.hpp"
#include "mixers/quadxcf.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/level.hpp"

static constexpr uint8_t SERIAL_PIN = 4;
static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 8.0f;

static SoftwareSerial softwareSerial;

hf::Hackflight h;

hf::DSMX_Receiver rc = hf::DSMX_Receiver(CHANNEL_MAP, DEMAND_SCALE);

hf::MixerQuadXCF mixer;

hf::RatePid ratePid = hf::RatePid(0.05f, 0.00f, 0.00f, 0.10f, 0.01f); 

hf::LevelPid levelPid = hf::LevelPid(0.20f);

// Timer task for DSMX serial receiver
static void receiverTask(void * params)
{
    while (true) {

        if (softwareSerial.available()) {
            rc.handleSerialEvent(softwareSerial.read(), micros());
        }

        delay(1);
    }
}

void setup(void)
{
    // Start software serial for the receiver
    softwareSerial.begin(115000, SERIAL_PIN);

    // Initialize Hackflight firmware
    h.init(new hf::TinyPico(), &rc, &mixer);

    // Add Rate and Level PID controllers
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);

    // Start the receiver timed task
    TaskHandle_t task;
    xTaskCreatePinnedToCore(receiverTask, "Task", 10000, NULL, 1, &task, 0);
}

void loop(void)
{
    h.update();
}
