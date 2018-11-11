/*
   ThingDev.ino : Hackflight sketch for SparkfunESP8266 ThingDev

   Solely for receiver prototyping
 
   Hardware support for SparkfunESP8266 ThingDev:

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
#include "boards/thingdev.hpp"
#include "mixers/quadx.hpp"
#include "pidcontrollers/level.hpp"
#include "receivers/esp8266.hpp"

static constexpr uint8_t CHANNEL_MAP[6] = {0,1,2,3,4,5};

hf::Hackflight h;

hf::ESP8266_Receiver rc = hf::ESP8266_Receiver(CHANNEL_MAP);

hf::MixerQuadX mixer;

hf::Rate ratePid = hf::Rate(0, 0, 0, 0, 0);

void setup(void)
{
     // Initialize Hackflight firmware
     h.init(new hf::SparkfunEsp8266ThingDev(), &rc, &mixer, &ratePid);
}

void loop(void)
{
    h.update();

    delay(10);
}
