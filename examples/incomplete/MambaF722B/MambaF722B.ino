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
#include <core/pids/angle.h>
#include <imu/softquat.h>
#include <esc/mock.h>

#include <vector>

#include <SPI.h>
#include <ICM42688.h>

static const uint8_t IMU_MOSI_PIN = PA7;
static const uint8_t IMU_MISO_PIN = PA6;
static const uint8_t IMU_SCLK_PIN = PA5;

static const uint8_t LED_PIN     = PC15; // PC15=blue; PC14=orange
static const uint8_t IMU_CS_PIN  = PA4;
static const uint8_t IMU_INT_PIN = PC4;

static SPIClass spi = SPIClass(IMU_MOSI_PIN, IMU_MISO_PIN, IMU_SCLK_PIN);

static ICM42688 icm(spi, IMU_CS_PIN);

static volatile bool gotInterrupt;

static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static Mixer mixer = QuadXbfMixer::make();

static SoftQuatImu imu(Imu::rotate270);

static std::vector<PidController *> pids = {&anglePid};

static MockEsc esc;

static Stm32F722Board board(imu, pids, mixer, esc, LED_PIN);

// IMU interrupt
static void handleImuInterrupt(void)
{
    imu.handleInterrupt(board.getCycleCounter());
}

void setup(void)
{
    Board::setInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);

    icm.begin();

    icm.setAccelODR(ICM42688::odr8k);
    icm.setGyroODR(ICM42688::odr8k);

    icm.enableDataReadyInterrupt(); 

    board.begin();
}

void loop(void)
{
    icm.getAGT();

    int16_t gyroCounts[3] = { 
        icm.getGyroX_count(),
        icm.getGyroY_count(),
        icm.getGyroZ_count()
    };

    int16_t accelCounts[3] = { 
        icm.getAccelX_count(),
        icm.getAccelY_count(),
        icm.getAccelZ_count()
    };

    board.step(gyroCounts, accelCounts);
}
