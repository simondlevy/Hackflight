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
#include <board/stm32/f/4/stm32f411.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <esc/mock.h>
#include <imu/mock.h>
#include <receiver/mock.h>

#include <vector>
using namespace std;

// IMU
static const uint8_t MOSI_PIN = PB_15;
static const uint8_t MISO_PIN = PB_14;
static const uint8_t SCLK_PIN = PB_13;

static const uint8_t CS_PIN   = PC_13;

//static const uint8_t LED_PIN  = PA13; // orange
static const uint8_t LED_PIN  = PA14; // blue

static Mixer mixer = QuadXbfMixer::make();

static MockEsc esc;

static MockReceiver rx;

static MockImu imu;

static vector<PidController *> pids = {};

static Stm32F411Board board(rx, imu, pids, mixer, esc, LED_PIN);

void setup(void)
{
    board.begin();
}

void loop(void)
{
    board.step();
}
