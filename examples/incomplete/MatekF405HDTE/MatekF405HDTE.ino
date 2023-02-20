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
#include <board/stm32f/stm32f4.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <core/pids/angle.h>
#include <debug.h>
#include <esc/dshot.h>
#include <imu/softquat.h>

#include <vector>

#include <SPI.h>
#include <ICM42688.h>

#include <dshot.h>
#include <stm32/stm32f4.h>

static const uint8_t LED_PIN     = PA13;

static const uint8_t IMU_CS_PIN  = PC14;
static const uint8_t IMU_INT_PIN = PC15;

static const uint8_t IMU_MOSI_PIN = PA7;
static const uint8_t IMU_MISO_PIN = PB4;
static const uint8_t IMU_SCLK_PIN = PA5;

static const uint8_t MOTOR1_PIN = PC9;
static const uint8_t MOTOR2_PIN = PC8;
static const uint8_t MOTOR3_PIN = PB15;

static const uint8_t MOTOR5_PIN = PB11;
static const uint8_t MOTOR6_PIN = PB10;
static const uint8_t MOTOR7_PIN = PB3;

static SPIClass spi = SPIClass(IMU_MOSI_PIN, IMU_MISO_PIN, IMU_SCLK_PIN);

static ICM42688 icm(spi, IMU_CS_PIN);

static Stm32F4Dshot dshot;

static DshotEsc esc = DshotEsc(&dshot);

static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static Mixer mixer = QuadXbfMixer::make();

static SoftQuatImu imu(Imu::rotate270Flip);

static std::vector<PidController *> pids = {&anglePid};

static Stm32F4Board board(imu, pids, mixer, esc, LED_PIN);

extern "C" void DMA2_Stream1_IRQHandler(void) 
{
    dshot.handleDmaIrqStream1();
}

extern "C" void DMA2_Stream2_IRQHandler(void) 
{
    dshot.handleDmaIrqStream2();
}

// IMU interrupt
static void handleImuInterrupt(void)
{
    board.handleImuInterrupt();
}

void setup(void)
{
    // Set up IMU interrupt
    board.setImuInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);

    icm.begin();

    board.begin();

    dshot.begin();

    dshot.addMotor(MOTOR1_PIN, 1); // PC9
    dshot.addMotor(MOTOR2_PIN, 1); // PC8
    dshot.addMotor(MOTOR3_PIN, 0); // PB15

    dshot.addMotor(MOTOR5_PIN, 0); // PB11
    dshot.addMotor(MOTOR6_PIN, 0); // PB10
    dshot.addMotor(MOTOR7_PIN, 0); // PB3
}

void loop(void)
{
    icm.getAGT();

    int16_t rawGyro[3] = { 
        icm.getGyroX_count(),
        icm.getGyroY_count(),
        icm.getGyroZ_count()
    };

    int16_t rawAccel[3] = { 
        icm.getAccelX_count(),
        icm.getAccelY_count(),
        icm.getAccelZ_count()
    };

    board.step(rawGyro, rawAccel);
}
