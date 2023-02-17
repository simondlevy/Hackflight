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
#include <esc/dshot.h>
#include <imu/softquat.h>

#include <sbus.h>

#include <vector>

#include <SPI.h>
#include <mpu6x00.h>

#include <dshot.h>
#include <stm32/stm32f4.h>

static const uint8_t LED_PIN     = PB5;
static const uint8_t IMU_CS_PIN  = PA4;
static const uint8_t IMU_INT_PIN = PC4;

static const uint8_t IMU_MOSI_PIN = PA7;
static const uint8_t IMU_MISO_PIN = PA6;
static const uint8_t IMU_SCLK_PIN = PA5;

static const uint8_t MOTOR1_PIN = PB_0;
static const uint8_t MOTOR2_PIN = PB_1;
static const uint8_t MOTOR3_PIN = PA_3;
static const uint8_t MOTOR4_PIN = PA_2;

static SPIClass spi = SPIClass(IMU_MOSI_PIN, IMU_MISO_PIN, IMU_SCLK_PIN);

static Mpu6x00 mpu = Mpu6x00(spi, IMU_CS_PIN);

static bfs::SbusRx rx(&Serial3);

static Stm32F4Dshot dshot;

static DshotEsc esc = DshotEsc(&dshot);

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
extern "C" void handleDmaIrq(uint8_t id)
{
    dshot.handleDmaIrq(id);
}

// IMU interrupt
static void handleImuInterrupt(void)
{
    board.handleImuInterrupt();
}

// Receiver interrupt
void serialEvent3(void)
{
    if (rx.Read()) {

        bfs::SbusData data = rx.data();

        board.setSbusValues((uint16_t *)data.ch, micros(), data.lost_frame);
    }
}

// Interupt from Skyranger
void serialEvent4(void)
{
    board.handleSkyrangerEvent(Serial4);
}

void setup(void)
{
    // Set up IMU interrupt
    board.setImuInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);

    // Start receiver UART
    Serial3.begin(100000, SERIAL_8E2);

    // Start Skyranger UART
    Serial4.begin(115200);

    mpu.begin();

    board.begin();

    dshot.begin();

    dshot.addMotor(MOTOR1_PIN, 0); 
    dshot.addMotor(MOTOR2_PIN, 0);
    dshot.addMotor(MOTOR3_PIN, 1);
    dshot.addMotor(MOTOR4_PIN, 1);
}

void loop(void)
{
    mpu.readSensor();

    int16_t rawGyro[3] = { mpu.getRawGyroX(), mpu.getRawGyroY(), mpu.getRawGyroZ() };
    int16_t rawAccel[3] = { mpu.getRawAccelX(), mpu.getRawAccelY(), mpu.getRawAccelZ() };

    // Support sending attitude data to Skyranger over Serial4
    board.step(rawGyro, rawAccel, Serial4);

}
