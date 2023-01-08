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
#include <board/stm32/stm32f4/stm32f405.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <receiver/mock.h>
#include <imu/real/softquat/mpu6x00.h>
#include <esc/mock.h>

#include <vector>
using namespace std;

// IMU
static const uint8_t MOSI_PIN = PA7;
static const uint8_t MISO_PIN = PA6;
static const uint8_t SCLK_PIN = PA5;
static const uint8_t CS_PIN   = PA4;
static const uint8_t EXTI_PIN = PC4;

static const uint8_t LED_PIN = PB5;

static Mixer mixer = QuadXbfMixer::make();

static MockReceiver rx;

static Mpu6x00 imu(MOSI_PIN, MISO_PIN, SCLK_PIN, CS_PIN, RealImu::rotate270);

static vector<PidController *> pids = {};

static MockEsc esc;

static Stm32F405Board board(rx, imu, pids, mixer, esc, LED_PIN);

// IMU interrupt
static void handleImuInterrupt(void)
{
    imu.handleInterrupt();
}

void setup(void)
{
    // Set up IMU interrupt
    Board::setInterrupt(EXTI_PIN, handleImuInterrupt, RISING);

    board.begin();
}

void loop(void)
{
    board.step();
}
