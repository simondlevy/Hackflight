/*
   Hackflight sketch for Ladybug Flight Controller with an SBUS receiver
 
   Additional libraries needed:

       https://github.com/simondlevy/EM7180
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/bolderflight/SBUS

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
#include "boards/realboards/arduino/ladybugfc.hpp"
#include "receivers/arduino/sbus2.hpp"
#include "mixers/quadxcf.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/level.hpp"

static constexpr uint8_t CHANNEL_MAP[6] = {0,1,2,3,4,5};
static constexpr float DEMAND_SCALE = 1.0f;

hf::Hackflight h;

hf::SBUS_Receiver rc = hf::SBUS_Receiver(CHANNEL_MAP, DEMAND_SCALE);

hf::MixerQuadXCF mixer;

hf::RatePid ratePid = hf::RatePid(
        0.225f,     // Gyro pitch/roll P
        0.001875f,  // Gyro pitch/roll I
        0.375f,     // Gyro pitch/roll D
        1.0625f,    // Gyro yaw P
        0.005625f); // Gyro yaw I

hf::LevelPid levelPid = hf::LevelPid(0.20f);

void setup(void)
{

    // Use pin A1 for LED on original LadybugFC (newer uses A4)
    h.init(new hf::LadybugFC(A1), &hf::ladybugIMU, &rc, &mixer, hf::ladybugFcMotors);

    // Add Rate, Level PID constrollers
    h.addPidController(&ratePid, 0);
    h.addPidController(&levelPid, 0);
}

void loop(void)
{
    h.update();
}
