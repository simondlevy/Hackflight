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
#include <boards/stm32/stm32f.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <escs/dshot/stm32f405.h>
#include <leds/mock.h>
#include <imus//mock.h>
#include <tasks/receivers/mock.h>

#include <vector>
using namespace std;

static AnglePidController _anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static vector<PidController *> _pids = {&_anglePid};

static Hackflight * _hf;
static Stm32F405DshotEsc * _esc;

static Mixer _mixer = QuadXbfMixer::make();

extern "C" void handleDmaIrq(uint8_t id)
{
    _esc->handleDmaIrq(id);
}

void setup(void)
{
    static MockReceiver rx;

    static Stm32FBoard board;

    vector<uint8_t> pins = {0x20, 0x21, 0x13, 0x12};

    static Stm32F405DshotEsc esc(pins);

    static MockImu imu;

    static MockLed led;

    static Hackflight hf(board, rx, imu, imuRotate270, _pids, _mixer, esc, led);

    _esc = &esc;
    _hf = &hf;

    hf.begin();
}

void loop(void)
{
    _hf->step();

    Serial.print(" 3: ");
    Serial.print(_esc->g_pinIndex);
    Serial.print(", ");
    Serial.println(_esc->g_portIndex);
}
