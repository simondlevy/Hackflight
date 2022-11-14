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
#include <alignment/rotate270.h>
#include <boards/stm32/stm32f405.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <escs/dshot.h>
#include <imus/real/softquat/mpu6000.h>
#include <tasks/receivers/sbus.h>

#include <vector>
using namespace std;

static const uint8_t MOSI_PIN = PA7;
static const uint8_t MISO_PIN = PA6;
static const uint8_t SCLK_PIN = PA5;
static const uint8_t CS_PIN   = PA4;
static const uint8_t EXTI_PIN = PC4;

//  --------------------------------- PB0    PB1  PA3   PA2
 static vector<uint8_t> MOTOR_PINS = {0x10, 0x11, 0x03, 0x02};

static const uint8_t LED_PIN  = PB5;

static SPIClass _spi(MOSI_PIN, MISO_PIN, SCLK_PIN);

static AnglePidController _anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static Stm32F405Board * _board;
static Mpu6000 * _imu;
static SbusReceiver * _rx;

static vector<PidController *> _pids = {&_anglePid};

extern "C" void handleDmaIrq(uint8_t id)
{
    _board->handleDmaIrq(id);
}

static void handleImuInterrupt(void)
{
    _imu->handleInterrupt();
}

void serialEvent3(void)
{
    _rx->handleEvent();
}

static Mixer _mixer = QuadXbfMixer::make();

void setup(void)
{
    pinMode(EXTI_PIN, INPUT);
    attachInterrupt(EXTI_PIN, handleImuInterrupt, RISING);  

    static SbusReceiver rx(Serial3);

    static Mpu6000 imu(_spi, CS_PIN);

    static DshotEsc esc(&MOTOR_PINS);

    static Stm32F405Board board(rx, imu, imuRotate270, _pids, _mixer, esc, LED_PIN);

    _board = &board;
    _rx = &rx;
    _imu = &imu;

    _board->begin();
}

void loop(void)
{
    _board->step();
}
