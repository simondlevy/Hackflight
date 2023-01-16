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
#include <board/stm32/ladybug.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <receiver/dsmx.h>
#include <unsafe/serial.h>

static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static vector<PidController *> pids = {&anglePid};

static Mixer mixer = QuadXbfMixer::make();

static DsmxReceiver rx;

static LadybugBoard board(rx, pids, mixer);

static void handleImuInterrupt(void)
{
    board.handleInterrupt();
}

void serialEvent1(void)
{
    handleReceiverSerialEvent(rx, Serial1);
}

void setup(void)
{
    Serial1.begin(115200);

    board.begin(handleImuInterrupt);
}

void loop(void)
{
    board.step();
}
