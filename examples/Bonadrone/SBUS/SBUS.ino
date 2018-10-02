/*
   SBUS.ino : Hackflight sketch for Bonadrone flight controller with SBUS receiver

   Additional libraries needed:

       https://github.com/simondlevy/LSM6DSM
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SBUSRX

   Hardware support for Bonadrone flight controller:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (c) 2018 Simon D. Levy

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
#include "boards/bonadrone.hpp"
#include "receivers/sbus.hpp"
#include "mixers/quadx.hpp"

// Change this as needed
#define SBUS_SERIAL Serial3

static constexpr uint8_t CHANNEL_MAP[6] = {0,1,2,3,4,5};

hf::Hackflight h;

hf::SBUS_Receiver rc = hf::SBUS_Receiver(CHANNEL_MAP, SERIAL_SBUS, &SBUS_SERIAL);

hf::MixerQuadX mixer;

hf::Stabilizer stabilizer = hf::Stabilizer(
                0.20f,      // Level P
                0.225,     // Gyro cyclic P
                0.001875f,  // Gyro cyclic I
                0.375f,     // Gyro cyclic D
                0,//1.0625f,    // Gyro yaw P
                0.005625f); // Gyro yaw I

void setup(void)
{
    h.init(new hf::Bonadrone(), &rc, &mixer, &stabilizer);
}

void loop(void)
{
    h.update();
}
