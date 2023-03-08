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
#include <boards/stm32f/stm32f4.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <core/pids/angle.h>
#include <escs/dshot.h>
#include <imus/softquat.h>

#include <dsmrx.h>

#include <vector>

#include <SPI.h>
#include <mpu6x00.h>

#include <dshot.h>
#include <stm32/stm32f4.h>

static const uint8_t LED_PIN     = PC14;
static const uint8_t IMU_CS_PIN  = PA4;
static const uint8_t IMU_INT_PIN = PB0;

static const uint8_t MOTOR1_PIN = PB3;
static const uint8_t MOTOR2_PIN = PB4;
static const uint8_t MOTOR3_PIN = PB6;
static const uint8_t MOTOR4_PIN = PB7;

static std::vector<uint8_t> motorPins = {MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN};

static SPIClass spi = SPIClass(
        Stm32FBoard::MOSI_PIN, Stm32FBoard::MISO_PIN, Stm32FBoard::SCLK_PIN);

static Mpu6000 mpu = Mpu6000(spi, IMU_CS_PIN);

static Dsm2048 rx;

static Stm32F4Dshot dshot;

static DshotEsc esc = DshotEsc(&dshot);

///////////////////////////////////////////////////////
static AnglePidController anglePid;
static Mixer mixer = QuadXbfMixer::make();
static SoftQuatImu imu(Imu::rotate180);
static std::vector<PidController *> pids = {&anglePid};
///////////////////////////////////////////////////////

static Stm32F4Board board(LED_PIN);

// Motor interrupt
extern "C" void DMA2_Stream1_IRQHandler(void) 
{
    dshot.handleDmaIrqStream1();
}

// IMU interrupt
static void handleImuInterrupt(void)
{
    board.handleImuInterrupt(imu);
}

// Receiver interrupt
void serialEvent1(void)
{
    while (Serial1.available()) {

        const auto usec = micros();

        rx.handleSerialEvent(Serial1.read(), usec);

        if (rx.gotNewFrame()) {

            uint16_t values[8] = {};
            rx.getChannelValues(values, 8);
            board.setDsmxValues(values, micros(), rx.timedOut(usec));
        }
    }
}

void setup(void)
{
    Serial1.begin(115200);

    spi.begin();

    mpu.begin();

    board.begin(imu, IMU_INT_PIN, handleImuInterrupt);

    dshot.begin(motorPins);
}

void loop(void)
{

    mpu.readSensor();

    int16_t rawGyro[3] = { mpu.getRawGyroX(), mpu.getRawGyroY(), mpu.getRawGyroZ() };
    int16_t rawAccel[3] = { mpu.getRawAccelX(), mpu.getRawAccelY(), mpu.getRawAccelZ() };

    board.step(imu, pids, mixer, esc, rawGyro, rawAccel);
}
