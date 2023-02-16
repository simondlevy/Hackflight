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
//#include <esc/dshot.h>
#include <esc/mock.h>
#include <imu/softquat.h>

#include <vector>

//#include <SPI.h>
//#include <mpu6x00.h>

//#include <stm32dshot.h>
//#include <dshot/stm32f4/stm32f405.h>

static const uint8_t LED_PIN     = PA13;
//static const uint8_t IMU_CS_PIN  = PC14;
//static const uint8_t IMU_INT_PIN = PC15;

//static const uint8_t IMU_MOSI_PIN = PA7;
//static const uint8_t IMU_MISO_PIN = PA6;
//static const uint8_t IMU_SCLK_PIN = PA5;

//static std::vector<uint8_t> MOTOR_PINS = {PC9, PC8, PB15, PA8};

//static SPIClass spi = SPIClass(IMU_MOSI_PIN, IMU_MISO_PIN, IMU_SCLK_PIN);

//static Mpu6x00 mpu = Mpu6x00(spi, IMU_CS_PIN);

//static Stm32F405Dshot dshot;

//static DshotEsc esc = DshotEsc(&dshot, &MOTOR_PINS);

static MockEsc esc;

static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static Mixer mixer = QuadXbfMixer::make();

static SoftQuatImu imu(Imu::rotate270);

static std::vector<PidController *> pids = {&anglePid};

static Stm32F4Board board(imu, pids, mixer, esc, LED_PIN);

// DSHOT timer interrupt
/*
extern "C" void handleDmaIrq(uint8_t id)
{
    dshot.handleDmaIrq(id);
}

// IMU interrupt
static void handleImuInterrupt(void)
{
    board.handleImuInterrupt();
}*/

void setup(void)
{
    // Set up IMU interrupt
    //board.setImuInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);

    //mpu.begin();

    board.begin();
}

void loop(void)
{
    //mpu.readSensor();

    int16_t rawGyro[3] = {/*mpu.getRawGyroX(), mpu.getRawGyroY(), mpu.getRawGyroZ()*/};
    int16_t rawAccel[3] = {/*mpu.getRawAccelX(), mpu.getRawAccelY(), mpu.getRawAccelZ()*/};

    board.step(rawGyro, rawAccel);

}
