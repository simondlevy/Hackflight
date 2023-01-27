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
#include <receiver/mock.h>
#include <imu/softquat/invensense/mpu6x00.h>
#include <esc/dshot.h>

#include <sbus.h>

#include <vector>

static const uint8_t LED_PIN     = PB5;
static const uint8_t IMU_CS_PIN  = PA4;
static const uint8_t IMU_INT_PIN = PC4;

static std::vector<uint8_t> MOTOR_PINS = {PB_0, PB_1, PA_3, PA_2};

static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static Mixer mixer = QuadXbfMixer::make();

static MockReceiver rx;

static Mpu6x00 imu(Imu::rotate270, IMU_CS_PIN);

static std::vector<PidController *> pids = {&anglePid};

static DshotEsc esc(MOTOR_PINS);

static Stm32F405Board board(rx, imu, pids, mixer, esc, LED_PIN);

// DSHOT timer interrupt
extern "C" void handleDmaIrq(uint8_t id)
{
    board.handleDmaIrq(id);
}

// IMU interrupt
static void handleImuInterrupt(void)
{
    board.handleImuInterrupt();
}

// Receiver interrupt
void serialEvent3(void)
{
}

// Interupt from Skyranger
void serialEvent4(void)
{
    board.handleSkyrangerEvent(Serial4);
}

void setup(void)
{
    // Set up IMU interrupt
    Board::setInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);

    // Start receiver UART
    Serial3.begin(100000, SERIAL_8E2);

    // Start Skyranger UART
    Serial4.begin(115200);

    board.begin();
}

void loop(void)
{
    // Support sending attitude data to Skyranger over Serial4
    board.step(Serial4);
}
