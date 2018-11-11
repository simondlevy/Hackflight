/*
   Superfly.ino : Hackflight sketch for SuperFly ESP8266 Hackable Flight Controller
 
   Additional libraries needed:

       https://github.com/simondlevy/EM7180
       https://github.com/simondlevy/CrossPlatformDataBus

   Hardware support for SuperFly ESP8266 Hackable flight controller:

       https://github.com/esp8266/Arduino#installing-with-boards-manager

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
#include "boards/superfly.hpp"
#include "mixers/quadx.hpp"
#include "pidcontrollers/level.hpp"
#include "receivers/esp8266.hpp"

static constexpr uint8_t CHANNEL_MAP[6] = {0,1,2,3,4,5};

hf::Hackflight h;

hf::ESP8266_Receiver rc = hf::ESP8266_Receiver(CHANNEL_MAP, "SuperFly");

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
     h.init(new hf::SuperFly(), &rc, &mixer, &ratePid);
}

void loop(void)
{
    h.update();

    delay(10);
}
