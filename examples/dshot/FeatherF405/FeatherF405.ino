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
#include <board/stm32/f/4/stm32f405.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <core/pids/angle.h>
#include <imu/softquat.h>
#include <esc/dshot.h>
#include <esc/mock.h>

#include <vector>

static const uint8_t LED_PIN = PC1;

// static std::vector<uint8_t> MOTOR_PINS = {PB_0, PB_1, PA_3, PA_2};
// static DshotEsc esc(MOTOR_PINS);

static MockEsc esc;

static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static Mixer mixer = QuadXbfMixer::make();

static SoftQuatImu imu(Imu::rotate270);

static std::vector<PidController *> pids = {&anglePid};

static Stm32F405Board board(imu, pids, mixer, esc, LED_PIN);

// DSHOT timer interrupt
/*
extern "C" void handleDmaIrq(uint8_t id)
{
    board.handleDmaIrq(id);
}*/

void setup(void)
{
    board.begin();
}

void loop(void)
{
    int16_t rawGyro[3] = {};
    int16_t rawAccel[3] = {};

    board.step(rawGyro, rawAccel);
}
