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
#include <board/stm32/stm32f4/stm32f405.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <receiver/sbus.h>
#include <imu/real/softquat/mpu6x00/arduino.h>
#include <esc/dshot.h>

#include <vector>
using namespace std;

// IMU
static const uint8_t MOSI_PIN = PA7;
static const uint8_t MISO_PIN = PA6;
static const uint8_t SCLK_PIN = PA5;
static const uint8_t CS_PIN   = PA4;
static const uint8_t EXTI_PIN = PC4;

static vector<uint8_t> MOTOR_PINS = {PB_0, PB_1, PA_3, PA_2};

static const uint8_t LED_PIN = PB5;

static SPIClass spi(MOSI_PIN, MISO_PIN, SCLK_PIN);

static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static Mixer mixer = QuadXbfMixer::make();

static SbusReceiver rx;

static ArduinoMpu6x00 imu(spi, RealImu::rotate270, CS_PIN);

static vector<PidController *> pids = {&anglePid};

static DshotEsc esc(&MOTOR_PINS);

static Stm32F405Board board(rx, imu, pids, mixer, esc, LED_PIN);

// DSHOT timer interrupt
extern "C" void handleDmaIrq(uint8_t id)
{
    board.handleDmaIrq(id);
}

// IMU interrupt
static void handleImuInterrupt(void)
{
    imu.handleInterrupt();
}

// Receiver interrupt
void serialEvent3(void)
{
    while (Serial3.available()) {
        rx.parse(Serial3.read());
    }
}

// Sensors interrupt
void serialEvent4(void)
{
    while (Serial4.available()) {
        board.parseSensors(Serial4.read());
    }
}

void setup(void)
{
    // Set up IMU interrupt
    pinMode(EXTI_PIN, INPUT);
    attachInterrupt(EXTI_PIN, handleImuInterrupt, RISING);  

    // Start receiver UART
    Serial3.begin(100000, SERIAL_8E2);

    // Start sensors UART
    Serial4.begin(115200);

    board.begin();
}

void loop(void)
{
    board.step();

    /*
    while (board.imuDataAvailable()) {
        Serial4.write(board.readImuData());
    }*/
}
