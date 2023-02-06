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
#include <board/teensy40.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <core/pids/angle.h>
#include <imu/softquat.h>
#include <esc/mock.h>

#include <vector>

#include <SPI.h>
#include <mpu6x00.h>

static const uint8_t IMU_CS_PIN  = 10;
static const uint8_t IMU_INT_PIN = 9;
static const uint8_t LED_PIN     = 0; // LED (pin 13) is taken by SPI SCLK

static Mpu6x00 mpu = Mpu6x00(IMU_CS_PIN);

static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static Mixer mixer = QuadXbfMixer::make();

static SoftQuatImu imu(Imu::rotate0);

static std::vector<PidController *> pids = {&anglePid};

static MockEsc esc;

static Teensy40 board(imu, pids, mixer, esc, LED_PIN);

// IMU interrupt
static void handleImuInterrupt() 
{
    board.handleImuInterrupt();
}

void setup() {

    Board::setInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);  

    SPI.begin();

    mpu.begin();

    board.begin();
}

void loop() 
{
    mpu.readSensor();

    int16_t rawGyro[3] = { mpu.getRawGyroX(), mpu.getRawGyroY(), mpu.getRawGyroZ() };

    int16_t rawAccel[3] = { mpu.getRawAccelX(), mpu.getRawAccelY(), mpu.getRawAccelZ() };

    board.step(rawGyro, rawAccel);

    Serial.println(rawGyro[2]);
}
