/*
   Copyright (c) 2024 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <dsmrx.hpp>

#include <closedloop.hpp>
#include <msp.hpp>

#include <ladybug/arduinostl.h>
#include <ladybug/ladybugfc.hpp>
#include <ladybug/mixers/fixedpitch/quadxbf.hpp>
#include <ladybug/pids/angle.hpp>

static Mixer mixer = QuadXbfMixer::make();
static AnglePidController anglePid;
static std::vector<PidController *> pids = {&anglePid};

static LadybugFC board;

static void handleImuInterrupt(void)
{
    board.handleImuInterrupt();
}

static uint32_t count;

// Receiver interrupt
void serialEvent2(void)
{
    count++;

    auto avail = Serial2.available();

    static uint8_t buf[256];

    Serial2.readBytes(buf, avail);

    for (uint8_t k=0; k<avail; ++k) {

        static Msp msp;

        auto msgtype = msp.parse(buf[k]);

        if (msgtype == 200) {
            Serial.println(msp.parseShort(0));
        }
    }
}

void setup(void)
{
    Serial2.begin(115200);

    board.begin(handleImuInterrupt);

}

void loop(void)
{
    board.step(pids, mixer);
}

namespace std {
    void __throw_bad_alloc() {
        while (true) {
            Serial.println("Unable to allocate memory");
            delay(500);
        }
    }
}
