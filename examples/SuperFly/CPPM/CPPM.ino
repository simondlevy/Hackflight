/*
   CPPM.ino : Hackflight sketch for SuperFly ESP8266 Hackable Flight Controller with a CPPM receiver
 
   Additional libraries needed:

       https://github.com/simondlevy/EM7180
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/CPPM

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
#include "receivers/cppm/arduino_cppm.hpp"
#include "receivers/dummy.hpp"

static constexpr uint8_t CHANNEL_MAP[6] = {0,1,2,3,4,5};

static const uint8_t RX_PIN = 12;

hf::Hackflight h;

//hf::CPPM_Receiver rc = hf::CPPM_Receiver(RX_PIN, CHANNEL_MAP);
hf::Dummy_Receiver rc;

hf::MixerQuadX mixer;

hf::Stabilizer stabilizer = hf::Stabilizer(
                0.20f,      // Level P
                0.225f,     // Gyro cyclic P
                0.001875f,  // Gyro cyclic I
                0.375f,     // Gyro cyclic D
                1.0625f,    // Gyro yaw P
                0.005625f); // Gyro yaw I

void setup(void)
{
    h.init(new hf::SuperFly(), &rc, &mixer, &stabilizer);
}

void loop(void)
{
    h.update();
}
