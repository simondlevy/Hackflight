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

#include <SPI.h>

#include <hackflight.h>
#include <board/stm32/f/stm32f722.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <esc/mock.h>
#include <imu/softquat/invensense/icm42688.h>
#include <receiver/mock.h>

#include <vector>
using namespace std;

// IMU
static const uint8_t MOSI_PIN = PA7;
static const uint8_t MISO_PIN = PA6;
static const uint8_t SCLK_PIN = PA5;
static const uint8_t CS_PIN   = PA4;
static const uint8_t EXTI_PIN = PC4;

static const uint8_t LED_PIN  = PC14;   // orange
//static const uint8_t LED_PIN  = PC15; // blue

static Mixer mixer = QuadXbfMixer::make();

static MockEsc esc;

static MockReceiver rx;

Icm42688 imu(MOSI_PIN, MISO_PIN, SCLK_PIN, CS_PIN, Imu::rotate0);

static vector<PidController *> pids = {};

static Stm32F722Board board(rx, imu, pids, mixer, esc, LED_PIN);

// IMU interrupt
static void handleImuInterrupt(void)
{
    imu.handleInterrupt(board.getCycleCounter());
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
