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
#include <board/stm32/stm32f4/stm32f411.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <esc/mock.h>
//#include <imu/real/softquat/bmi270.h>
#include <imu/mock.h>
#include <task/receiver/real/sbus.h>

// IMU
//static const uint8_t MOSI_PIN = PA7; // PB15
//static const uint8_t MISO_PIN = PA6; // PB14
//static const uint8_t SCLK_PIN = PA5; // PB13
//static const uint8_t CS_PIN   = PA4;
//static const uint8_t EXTI_PIN = PA1;

#include <vector>
using namespace std;

//static const uint8_t LED_PIN  = PC13; // orange
static const uint8_t LED_PIN  = PC14; // blue

//static SPIClass _spi(MOSI_PIN, MISO_PIN, SCLK_PIN);

static AnglePidController _anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static vector<PidController *> _pids = {&_anglePid};

static Stm32F411Board * _board;
//static Bmi270 * _imu;

static SbusReceiver _rx;

static Mixer _mixer = QuadXbfMixer::make();

void serialEvent2(void)
{
    _rx.read(Serial2);
}

void setup(void)
{
    //static Bmi270 imu(RealImu::rotate270, _spi, CS_PIN);

    static MockImu imu;

    static MockEsc esc;

    static Stm32F411Board board(_rx, imu, _pids, _mixer, esc, LED_PIN);

    _board = &board;
    //_imu = &imu;

    Serial2.begin(100000, SERIAL_8E2);

    _board->begin();
}

void loop(void)
{
    _board->step();

    //Serial.println(_imu->id, HEX);
}