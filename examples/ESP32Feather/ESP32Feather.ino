/*
   ESP32Feather.ino : Hackflight sketch for ESP32 Feather

   Copyright (c) 2019 Simon D. Levy

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
#include "boards/arduino/esp32feather.hpp"
#include "receivers/mock.hpp"
#include "mixers/quadxcf.hpp"

hf::Hackflight h;

hf::MockReceiver rc;

hf::MixerQuadXCF mixer;

hf::Rate ratePid = hf::Rate(0, 0, 0, 0, 0);

void setup(void)
{
     // Initialize Hackflight firmware
     h.init(new hf::ESP32FeatherBoard(), &rc, &mixer, &ratePid);
}

void loop(void)
{
    h.update();

    delay(10);
}
