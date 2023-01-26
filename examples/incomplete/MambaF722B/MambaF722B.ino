/*
   Copyright (c) 2023 Simon D. Levy

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
#include <board/stm32/f/stm32f722.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <esc/mock.h>
// #include <imu/softquat/invensense/icm42688.h>
#include <imu/softquat/invensense/mock.h>
#include <receiver/mock.h>

#include <vector>

static const uint8_t LED_PIN     = PC15; // PC15=blue; PC14=orange
static const uint8_t IMU_CS_PIN  = PA4;
static const uint8_t IMU_INT_PIN = PC4;

static Mixer mixer = QuadXbfMixer::make();

static MockEsc esc;

static MockReceiver rx;

// Icm42688 imu(Imu::rotate270, IMU_CS_PIN);
MockImu imu;

static std::vector<PidController *> pids = {};

static Stm32F722Board board(rx, imu, pids, mixer, esc, LED_PIN);

// IMU interrupt
static void handleImuInterrupt(void)
{
    imu.handleInterrupt(board.getCycleCounter());
}

void setup(void)
{
    // Set up IMU interrupt
    //Board::setInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);

    board.begin();
}

void loop(void)
{
    board.step();
}
