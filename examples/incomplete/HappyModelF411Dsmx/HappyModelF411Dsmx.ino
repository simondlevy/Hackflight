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
#include <esc/dshot.h>

#include <dsmrx.h>

#include <vector>

#include <SPI.h>
#include <ICM20689.h>

static const uint8_t IMU_MOSI_PIN = PA7;
static const uint8_t IMU_MISO_PIN = PA6;
static const uint8_t IMU_SCLK_PIN = PA5;

static const uint8_t LED_PIN     = PC13;
static const uint8_t IMU_CS_PIN  = PA4;
static const uint8_t IMU_INT_PIN = PA1;

static SPIClass spi = SPIClass(IMU_MOSI_PIN, IMU_MISO_PIN, IMU_SCLK_PIN);

static ICM20689 icm(spi, IMU_CS_PIN);

static std::vector <uint8_t> MOTOR_PINS = {PB_10, PB_7, PB_7, PB_8};

static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static Mixer mixer = QuadXbfMixer::make();

static Dsm2048 rx;

static SoftQuatImu imu(Imu::rotate90);

static std::vector<PidController *> pids = {&anglePid};

static DshotEsc esc(MOTOR_PINS);

static Stm32F411Board board(imu, pids, mixer, esc, LED_PIN);

// Motor interrupt
extern "C" void handleDmaIrq(void)
{
    board.handleDmaIrq(0);
}

// IMU interrupt
static void handleImuInterrupt() 
{
    board.handleImuInterrupt();
}

// Receiver interrupt
void serialEvent2(void)
{
    while (Serial2.available()) {

        rx.handleSerialEvent(Serial2.read(), micros());

        if (rx.gotNewFrame()) {

            uint16_t values[8] = {};
            rx.getChannelValues(values, 8);
            board.setDsmxValues(values, micros());
        }
    }
}

void setup() {

    board.setImuInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);  

    Serial2.begin(115200);

    icm.begin();

    icm.enableDataReadyInterrupt();

    icm.setDlpfBandwidth(ICM20689::DLPF_BANDWIDTH_21HZ);

    icm.setSrd(19);

    board.begin();
}

void loop() 
{
    icm.readSensor();

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
