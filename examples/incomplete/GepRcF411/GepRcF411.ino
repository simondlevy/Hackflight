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
#include <core/pids/angle.h>
#include <imu/softquat.h>
#include <esc/mock.h>

#include <vector>

#include <SPI.h>
#include <BMI270.h>

static const uint8_t IMU_MOSI_PIN = PA7;
static const uint8_t IMU_MISO_PIN = PA6;
static const uint8_t IMU_SCLK_PIN = PA5;

static const uint8_t LED_PIN     = PC13;
static const uint8_t IMU_CS_PIN  = PA4;
static const uint8_t IMU_INT_PIN = PA1;

static SPIClass spi = SPIClass(IMU_MOSI_PIN, IMU_MISO_PIN, IMU_SCLK_PIN);

static BMI270 bmi = BMI270(spi, 
        IMU_CS_PIN,
        BMI270::ACCEL_RANGE_2_G,
        BMI270::ACCEL_ODR_100_HZ,
        BMI270::GYRO_RANGE_2000_DPS,
        BMI270::GYRO_ODR_3200_HZ);

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

static Stm32F411Board board(imu, pids, mixer, esc, LED_PIN);

// IMU interrupt
static void handleImuInterrupt() 
{
    // board.handleImuInterrupt();
}

void setup() {

    //Board::setInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);  

    spi.begin();

    bmi.begin();

    board.begin();
}

void loop() 
{
    if (Serial.available() && Serial.read() == 'R') {
        board.reboot();
    }

    //int16_t rawGyro[3] = {};
    //int16_t rawAccel[3] = {};
    //board.step(rawGyro, rawAccel);
}
