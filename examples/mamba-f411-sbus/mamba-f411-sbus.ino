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
#include <core/clock.h>
#include <core/mixers/fixedpitch/quadxbf.h>

#include <escs/mock.h>
#include <imus/mock.h>
#include <tasks/receivers/mock.h>

#include <leds/arduino.h>

#include <vector>
using namespace std;

static AnglePidController _anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static MockEsc _esc;
static MockImu _imu;
static MockReceiver _rx;

vector<PidController *> _pids = {&_anglePid};

static Mixer _mixer = QuadXbfMixer::make();

static ArduinoLed _led(PC14); // pin

static Hackflight _hf(_rx, _imu, imuRotate270, _pids, _mixer, _esc, _led);

uint32_t systemGetCycleCounter(void)
{
    return DWT->CYCCNT;
}

static const uint8_t LED = PC14;
static const uint16_t DELAY = 1000;

void setup() 
{
    pinMode(LED, OUTPUT);
}

void loop() 
{
    digitalWrite(LED, HIGH);   
    delay(DELAY);              
    digitalWrite(LED, LOW);    
    delay(DELAY);             
}
