/*
   Use TinyPICO as a proxy between DSMX receiver in and SBUS out

   Additional libraries needed:

       https://github.com/simondlevy/DSMRX


   Copyright (c) 2020 Simon D. Levy

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

#include "hackflight.hpp"
#include "boards/realboards/tinypico.hpp"
#include "receivers/arduino/dsmx.hpp"
#include "actuators/rxproxies/sbus.hpp"
#include "actuators/rxproxies/mock.hpp"

static const uint8_t SERIAL1_RX =  4;
static const uint8_t SERIAL1_TX = 15; // unused

static const uint8_t SERIAL2_RX = 16; // unused
static const uint8_t SERIAL2_TX = 14;

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

static constexpr float DEMAND_SCALE = 8.0f;

hf::Hackflight h;

hf::DSMX_Receiver dsmx_in = hf::DSMX_Receiver(CHANNEL_MAP, DEMAND_SCALE);  

hf::SbusProxy sbus_out;

// Timer task for DSMX serial receiver
static void receiverTask(void * params)
{
    while (true) {

        if (Serial1.available()) {
            dsmx_in.handleSerialEvent(Serial1.read(), micros());
        }

        delay(1);
    }
}


void setup(void)
{
    // Start DSMX receiver input on Serial1
    Serial1.begin(115000, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);

    // Initialize Hackflight firmware
    h.init(new hf::TinyPico(), &dsmx_in, &sbus_out);

    // Start the receiver timed task
    TaskHandle_t task;
    xTaskCreatePinnedToCore(receiverTask, "Task", 10000, NULL, 1, &task, 0);
}

void loop(void)
{
    h.update();
}
