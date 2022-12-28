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
#include <receiver/real/dsmx.h>

static AnglePidController _anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static vector<PidController *> _pids = {&_anglePid};

static Mixer _mixer = QuadXbfMixer::make();

static LadybugBoard * _board;

static DsmxReceiver _rx;

static void handleImuInterrupt(void)
{
    _board->handleInterrupt();
}

void serialEvent1(void)
{
    _rx.read(Serial1);
}

void setup(void)
{
    pinMode(LadybugBoard::IMU_INTERRUPT_PIN, INPUT);
    attachInterrupt(LadybugBoard::IMU_INTERRUPT_PIN, handleImuInterrupt, RISING);  

    static LadybugBoard board(_rx, _pids, _mixer);

    _board = &board;

    Serial1.begin(115200);

    _board->begin();
}

void loop(void)
{
    _board->step();
}
