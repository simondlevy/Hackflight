/*
   DSMX.ino : Hackflight sketch for Ladybug Flight Controller with Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/EM7180
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Hardware support for Ladybug flight controller:

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
#include "boards/ladybug.hpp"
#include "receivers/dsmx.hpp"
#include "mixers/quadx.hpp"
#include "pidcontrollers/level.hpp"

constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

hf::Hackflight h;

hf::DSMX_Receiver rc = hf::DSMX_Receiver(CHANNEL_MAP);  

hf::MixerQuadX mixer;

hf::Rate ratePid = hf::Rate(
        0.225f,     // Gyro pitch/roll P
        0.001875f,  // Gyro pitch/roll I
        0.375f,     // Gyro pitch/roll D
        1.0625f,    // Gyro yaw P
        0.005625f); // Gyro yaw I

hf::Level level = hf::Level(0.20f);

void setup(void)
{
    // Add Level PID for aux switch position 1
    h.addPidController(&level, 1);

    // Initialize Hackflight firmware
    h.init(new hf::Ladybug(), &rc, &mixer, &ratePid);
}

void loop(void)
{
    h.update();
}
