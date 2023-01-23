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
#include <receiver/sbus.h>
#include <imu/softquat/invensense/mpu6x00.h>
#include <esc/mock.h>

#include <vector>
using namespace std;

static const uint8_t IMU_CS_PIN   = PA4;
static const uint8_t IMU_INT_PIN = PA1;
static const uint8_t LED_PIN  = PC13; 

static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static vector<PidController *> _pids = {&anglePid};

static Mixer mixer = QuadXbfMixer::make();

static SbusReceiver rx;

static Mpu6x00 imu(Imu::rotate180, IMU_CS_PIN);

static vector<PidController *> pids = {&anglePid};

static MockEsc esc;

static Stm32F411Board board(rx, imu, pids, mixer, esc, LED_PIN);

// IMU interrupt
static void handleImuInterrupt(void)
{
    imu.handleInterrupt(board.getCycleCounter());
}

// Receiver interrupt
void serialEvent1(void)
{
    Board::handleReceiverSerialEvent(rx, Serial1);
}

void setup(void)
{
    // Set up IMU interrupt
    Board::setInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);  

    // Start receiver UART
    Serial1.begin(100000, SERIAL_8E2);

    board.begin();
}

void loop(void)
{
    board.step();
}
