/*
   Hackflight sketch for Ladybug Flight Controller with Spektrum DSMX receiver

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
#include "boards/arduino/ladybug.hpp"
#include "receivers/arduino/dsmx.hpp"
#include "mixers/quadxcf.hpp"
#include "pidcontrollers/acro.hpp"
#include "pidcontrollers/level.hpp"
#include "pidcontrollers/yaw.hpp"

constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

hf::Hackflight h;

hf::DSMX_Receiver rc = hf::DSMX_Receiver(CHANNEL_MAP);  

hf::MixerQuadXCF mixer;

hf::AcroPid acroPid = hf::AcroPid(0.225, 0.001875, 0.375); 

hf::LevelPid levelPid = hf::LevelPid(0.20);

hf::YawPid yawPid = hf::YawPid(1.0625, 0.005625f);

void setup(void)
{
    // Initialize Hackflight firmware
    h.init(new hf::Ladybug(), &rc, &mixer);

    // Add Level, Yaw PIDs for aux switch position 0
    h.addPidController(&yawPid, 0);
    h.addPidController(&levelPid, 0);
}

void loop(void)
{
    h.update();
}
