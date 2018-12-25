/*
   Mock.ino : Hackflight sketch for Tlera Dragonfly with mock board and receiver

   Solely for receiver prototyping
 
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
#include "boards/dragonfly.hpp"
#include "mixers/quadx.hpp"
#include "receivers/mock.hpp"
#include "sensors/mspsensors/rangeandflow.hpp"

static constexpr uint8_t CHANNEL_MAP[6] = {0,1,2,3,4,5};

hf::Hackflight h;

hf::MockReceiver rc;

hf::MixerQuadX mixer;

hf::Rate ratePid = hf::Rate(0, 0, 0, 0, 0);

hf::RangeAndFlow rangeAndFlow;

void setup(void)
{
     // Initialize Hackflight firmware (LED 25, inverted)
     h.init(new hf::DragonflyBoard(), &rc, &mixer, &ratePid);

     // Set up to receive telemetry over Serial1
     Serial1.begin(115200);
}

void loop(void)
{
    h.update();

    delay(10);
}
