/*
   Copyright (c) 2022 Simon D. Levy

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

#include <hackflight.h>
#include <boards/ladybug.h>
#include <core/pids/angle.h>
#include <core/mixers/fixedpitch/quadxbf.h>

#include <dsmrx.h>

#include <vector>

static Dsm2048 rx;

static Mixer mixer = QuadXbfMixer::make();
static AnglePidController anglePid;
static std::vector<PidController *> pids = {&anglePid};

static LadybugBoard board;

static void handleImuInterrupt(void)
{
    board.handleImuInterrupt();
}

// Receiver interrupt
void serialEvent1(void)
{
    while (Serial1.available()) {

        const auto usec = micros();

        rx.handleSerialEvent(Serial1.read(), usec);

        if (rx.gotNewFrame()) {

            uint16_t values[8] = {};
            rx.getChannelValues(values, 8);
            board.setDsmxValues(values, micros(), rx.timedOut(usec));
        }
    }
}

void setup(void)
{
    Serial1.begin(115200);

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
