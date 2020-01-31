/*
   Hackflight sketch for Teensy 4.0 board with Ultimate Sensor Fusion Solution IMU and DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Copyright (c) 2020 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>

#include "hackflightfull.hpp"
#include "boards/realboards/arduino/teensy40.hpp"
#include "imus/mock.hpp"
#include "receivers/mock.hpp"
#include "motors/mock.hpp"
#include "mixers/quadxcf.hpp"

hf::HackflightFull h;

hf::MockIMU imu;

hf::MockReceiver rc;

hf::MixerQuadXCF mixer;

hf::MockMotor motor1;
hf::MockMotor motor2;
hf::MockMotor motor3;
hf::MockMotor motor4;

hf::Motor * motors[4] = { &motor1, &motor2, &motor3, &motor4 };

void setup(void)
{
    // Initialize Hackflight firmware
    h.init(new hf::Teensy40(), &imu, &rc, &mixer, motors);
}

void loop(void)
{
    h.update();
}
