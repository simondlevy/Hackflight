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
#include <boards/stm32/stm32f.h>
//#include <core/clock.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <escs/dshot/stm32f405.h>
#include <leds/real.h>
#include <imus/real/softquat/mpu6000.h>
#include <tasks/receivers/real/sbus.h>

#include <vector>
using namespace std;

// See https://github.com/betaflight/unified-targets/blob/master/configs/default/BEFH-BETAFPVF405.config
static const uint8_t CS_PIN   = PA4;
static const uint8_t LED_PIN  = PB5;
static const uint8_t EXTI_PIN = PC4;

static AnglePidController _anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static Stm32F405DshotEsc * _esc;
static Hackflight * _hf;
static Mpu6000 * _imu;
static SbusReceiver * _rx;

static vector<PidController *> _pids = {&_anglePid};

extern "C" void handleDmaIrq(uint8_t id)
{
    _esc->handleDmaIrq(id);
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

    static Stm32FBoard board;

    vector<uint8_t> pins = {0x20, 0x21, 0x13, 0x12};

    static Stm32F405DshotEsc esc(board, pins);

    static Mpu6000 imu(CS_PIN, &board);

    static RealLed led(LED_PIN);

    static Hackflight hf(board, rx, imu, imuRotate270, _pids, _mixer, esc, led);

    _rx = &rx;
    _imu = &imu;
    _esc = &esc;
    _hf = &hf;

    hf.begin();
}

void loop(void)
{
    _hf->step();
}
