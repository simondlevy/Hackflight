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
#include <boards/stm32/ladybug.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <escs/mock.h>
#include <imus/real/usfs.h>
#include <leds//real.h>
#include <tasks/receivers/mock.h>
#include <alignment/rotate0.h>

static const uint8_t IMU_INTERRUPT_PIN = 0x0C;
static const uint8_t LED_PIN = 0x12;

static AnglePidController _anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static vector<PidController *> _pids = {&_anglePid};

static Mixer _mixer = QuadXbfMixer::make();

static Hackflight * _hf;

static UsfsImu * _imu;

static void handleImuInterrupt(void)
{
    //_imu->handleInterrupt();
}

void setup(void)
{
    //pinMode(IMU_INTERRUPT_PIN, INPUT);
    //attachInterrupt(IMU_INTERRUPT_PIN, handleImuInterrupt, RISING);  

    static LadybugBoard board;
    static MockReceiver rx;
    static MockEsc esc;

    static RealLed led(LED_PIN);

    static UsfsImu imu;

    static Hackflight hf(board, rx, imu, imuRotate0, _pids, _mixer, esc, led);

    _hf = &hf;
    _imu = &imu;

    hf.begin();
}

void loop(void)
{
    _hf->step();
}
