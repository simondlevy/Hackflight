/*
   Hackflight sketch for Ladybug dev board with mock receiver and software quaternion

   Additional libraries needed:

       https://github.com/simondlevy/MPU
       https://github.com/simondlevy/CrossPlatformDataBus

   Hardware support for SMT32L4:

       https://github.com/simondlevy/grumpyoldpizza

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

#include "hackflight.hpp"
#include "boards/realboards/arduino/ladybug.hpp"
#include "receivers/mock.hpp"
#include "imus/softquats/mpu9250.hpp"
#include "actuators/mixers/quadxcf.hpp"
#include "motors/mock.hpp"

hf::Hackflight h;

hf::MPU9250SoftwareQuaternionIMU imu;

hf::MockReceiver rc;

hf::MixerQuadXCF mixer;

hf::MockMotor motor1;
hf::MockMotor motor2;
hf::MockMotor motor3;
hf::MockMotor motor4;

hf::Motor * motors[4] = { &motor1, &motor2, &motor3, &motor4 };

void setup(void)
{
    h.init(new hf::Ladybug(), &imu, &rc, &mixer, motors);
}

void loop(void)
{
    h.update();
}
