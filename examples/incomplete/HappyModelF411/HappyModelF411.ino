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
#include <board/stm32/f/4/stm32f411.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <receiver/mock.h>
#include <imu/softquat/invensense/icm20689.h>
#include <esc/mock.h>

#include <vector>

static const uint8_t LED_PIN     = PC13;
static const uint8_t IMU_CS_PIN  = PA4;
static const uint8_t IMU_INT_PIN = PA1;

static Mixer mixer = QuadXbfMixer::make();

static MockReceiver rx;

static Icm20689 imu(Imu::rotate90, IMU_CS_PIN);

static MockEsc esc;

static std::vector<PidController *> pids = {};

static Stm32F411Board board(rx, imu, pids, mixer, esc, LED_PIN);

// IMU interrupt
static void handleImuInterrupt(void)
{
    board.handleImuInterrupt();
}

void setup(void)
{
    Board::setInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);  

    board.begin();
}

void loop(void)
{
    board.step();
}
