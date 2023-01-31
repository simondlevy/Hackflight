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
#include <core/pids/angle.h>
#include <imu/softquat.h>
#include <esc/dshot.h>

#include <dsmrx.h>

#include <vector>

#include <SPI.h>
#include <mpu6x00.h>

static const uint8_t LED_PIN     = PC14;
static const uint8_t IMU_CS_PIN  = PA4;
static const uint8_t IMU_INT_PIN = PB0;

static const uint8_t IMU_MOSI_PIN = PA7;
static const uint8_t IMU_MISO_PIN = PA6;
static const uint8_t IMU_SCLK_PIN = PA5;

static SPIClass spi = SPIClass(IMU_MOSI_PIN, IMU_MISO_PIN, IMU_SCLK_PIN);

static Mpu6x00 mpu = Mpu6x00(spi, IMU_CS_PIN);

static std::vector <uint8_t> MOTOR_PINS = {PB3, PB4, PB6, PB7};

static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static Mixer mixer = QuadXbfMixer::make();

static Dsm2048 rx;

static SoftQuatImu imu(Imu::rotate180);

static std::vector<PidController *> pids = {&anglePid};

static DshotEsc esc(MOTOR_PINS);

static Stm32F411Board board(imu, pids, mixer, esc, LED_PIN);

// Motor interrupt
extern "C" void handleDmaIrq(void)
{
    board.handleDmaIrq(0);
}

// IMU interrupt
static void handleImuInterrupt(void)
{
    board.handleImuInterrupt();
}

// Receiver interrupt
void serialEvent1(void)
{
    while (Serial1.available()) {

        rx.handleSerialEvent(Serial1.read(), micros());

        if (rx.gotNewFrame()) {

            uint16_t values[8] = {};
            rx.getChannelValues(values, 8);
            board.setDsmxValues(values, micros());
        }
    }
}

void setup(void)
{
    Board::setInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);  

    Serial1.begin(115200);

    mpu.begin();

    board.begin();
}

void loop(void)
{
    int16_t rawGyro[3] = {};
    int16_t rawAccel[3] = {};
    mpu.readData();
    mpu.getRawGyro(rawGyro[0], rawGyro[1], rawGyro[2]);
    mpu.getRawAccel(rawAccel[0], rawAccel[1], rawAccel[2]);

    board.step(rawGyro, rawAccel);
}
