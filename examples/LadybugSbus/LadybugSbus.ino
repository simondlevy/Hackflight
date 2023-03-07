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
#include <core/mixers/fixedpitch/quadxbf.h>
#include <core/pids/angle.h>

#include <sbus.h>

#include <vector>

static AnglePidController anglePid;

static std::vector<PidController *> pids = {&anglePid};

static Mixer mixer = QuadXbfMixer::make();

static bfs::SbusRx rx(&Serial1);

static LadybugBoard board(pids, mixer);

static void handleImuInterrupt(void)
{
    board.handleImuInterrupt();
}

// Receiver interrupt
void serialEvent1(void)
{
    if (rx.Read()) {

        bfs::SbusData data = rx.data();

        board.setSbusValues((uint16_t *)data.ch, micros(), data.lost_frame);
    }
}

void setup(void)
{
    Serial1.begin(100000, SERIAL_SBUS);

    board.begin(handleImuInterrupt);
}

void loop(void)
{
    board.step();
}
