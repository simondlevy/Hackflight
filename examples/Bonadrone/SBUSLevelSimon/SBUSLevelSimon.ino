/*
   SBUSLevelSimon.ino : Hackflight sketch for Bonadrone flight controller with SBUS receiver and
   Simon's TAER channel map and RX connection

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
#include "boards/arduino/bonadrone.hpp"
#include "receivers/sbus.hpp"
#include "mixers/quadxcf.hpp"

#include "pidcontrollers/level.hpp"

// Change this as needed
#define SBUS_SERIAL Serial3

static constexpr uint8_t CHANNEL_MAP[6] = {0,1,2,3,4,5};

hf::Hackflight h;

hf::SBUS_Receiver rc = hf::SBUS_Receiver(CHANNEL_MAP, SERIAL_SBUS, &SBUS_SERIAL);

hf::MixerQuadXCF mixer;

hf::Rate ratePid = hf::Rate(
        0.10f,  // PitchRoll P
        0.01f,  // PitchRoll I
        0.05f,  // PitchRoll D
        0.10,   // Yaw P
        0.01f,  // Yaw I
        8.58f); // Demands to rate

hf::Level level = hf::Level(0.25f);

void setup(void)
{

    // Aux switch 1 for Level mode
    h.addPidController(&level, 1);

    h.init(new hf::BonadroneMultiShot(), &rc, &mixer, &ratePid);
}

void loop(void)
{
    h.update();
}
