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
#include <alignment/rotate270.h>
#include <boards/stm32/stm32f4.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <escs/mock.h>
#include <imus//mock.h>
#include <tasks/receivers/mock.h>

#include <vector>
using namespace std;

static const uint8_t LED_PIN  = PC14;
static const uint8_t EXTI_PIN = PA1;

static AnglePidController _anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static vector<PidController *> _pids = {&_anglePid};

static Stm32F4Board * _board;

static Mixer _mixer = QuadXbfMixer::make();

extern "C" void handleDmaIrq(uint8_t id)
{
    _board->handleDmaIrq(id);
}

static void handleImuInterrupt(void)
{
    //_imu->handleInterrupt();
}

void setup(void)
{
    pinMode(EXTI_PIN, INPUT);
    attachInterrupt(EXTI_PIN, handleImuInterrupt, RISING);  

    static MockReceiver rx;

    static MockImu imu;

    static MockEsc esc;

    static Stm32F4Board board(rx, imu, imuRotate270, _pids, _mixer, esc, LED_PIN);

    _board = &board;

    _board->begin();
}

void loop(void)
{
    _board->step();
}
