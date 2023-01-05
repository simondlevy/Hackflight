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
#include <msp/arduino.h>
#include <board/stm32/stm32f745.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <esc/mock.h>
#include <imu/mock.h>
#include <receiver/mock.h>

#include <vector>
using namespace std;

static const uint8_t LED_PIN  = PA_2;

static ArduinoMsp _msp;

static MockReceiver _rx;

static MockImu _imu;

static vector<PidController *> _pids = {};

static Mixer _mixer = QuadXbfMixer::make();

static MockEsc _esc;

static Stm32F745Board _board(_msp, _rx, _imu, _pids, _mixer, _esc, LED_PIN);

void setup(void)
{
    _board.begin();
}

void loop(void)
{
    _board.step();
}
