/*
   Test TinyPICO running DSHOT600 ESC protocol


   Copyright (c) 2020 Simon D. Levy

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

#include "hackflight.hpp"
#include "boards/realboards/tinypico.hpp"
#include "receivers/mock.hpp"
#include "imus/mock.hpp"
#include "motors/esp32dshot600.hpp"
#include "actuators/mixers/quadxcf.hpp"

static const uint8_t PINS[4] = {25, 26, 27, 15};

hf::Esp32DShot600 motors = hf::Esp32DShot600(PINS, 4);

hf::Hackflight h;

hf::MockReceiver rc;

hf::MockIMU imu;

hf::MixerQuadXCF mixer;

void setup(void)
{
    // Initialize Hackflight firmware
    h.init(new hf::TinyPico(), &imu, &rc, &mixer, &motors);

}

void loop(void)
{
    h.update();
}
