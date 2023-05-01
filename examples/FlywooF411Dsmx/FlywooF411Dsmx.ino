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
#include <escs/mock.h>
#include <imus/softquat.h>

#include <dsmrx.h>

#include <vector>

#include <SPI.h>
#include <mpu6x00.h>

#include <dshot_stm32f4.h>

static const uint8_t LED_PIN     = PC13;
static const uint8_t IMU_CS_PIN  = PA4;
static const uint8_t IMU_INT_PIN = PB2;

static SPIClass spi = SPIClass(
        Stm32FBoard::MOSI_PIN, Stm32FBoard::MISO_PIN, Stm32FBoard::SCLK_PIN);

static Mpu6500 mpu = Mpu6500(spi, IMU_CS_PIN);

static Dsm2048 rx;

static MockEsc esc;

///////////////////////////////////////////////////////
static AnglePidController anglePid;
static Mixer mixer = QuadXbfMixer::make();
static SoftQuatImu imu(Imu::rotate0Flip);
static std::vector<PidController *> pids = {&anglePid};
///////////////////////////////////////////////////////

static Stm32F4Board board(LED_PIN);

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
    // Start receiver UART
    Serial1.begin(115200);

    mpu.begin();

    board.begin(imu, IMU_INT_PIN, handleImuInterrupt);
}

void loop(void)
{

    mpu.readSensor();

    int16_t rawGyro[3] = { mpu.getRawGyroX(), mpu.getRawGyroY(), mpu.getRawGyroZ() };
    int16_t rawAccel[3] = { mpu.getRawAccelX(), mpu.getRawAccelY(), mpu.getRawAccelZ() };

    board.step(imu, pids, mixer, esc, rawGyro, rawAccel);
}
