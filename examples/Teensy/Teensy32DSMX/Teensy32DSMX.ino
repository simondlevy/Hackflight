/*
   Teensy32DSMX.ino : Hackflight sketch for Teensy 3.2 dev-board with Spektrum DSMX receiver

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
#include "boards/teensy32.hpp"
#include "receivers/serial/arduino_dsmx.hpp"
#include "mixers/quadx.hpp"

constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

hf::Hackflight h;

hf::DSMX_Receiver rc = hf::DSMX_Receiver(CHANNEL_MAP);

hf::MixerQuadX mixer;

hf::Stabilizer stabilizer = hf::Stabilizer(
                0,//0.20f,      // Level P
                0,//0.225f,     // Gyro cyclic P
                0,//0.001875f,  // Gyro cyclic I
                0,//0.375f,     // Gyro cyclic D
                0,//1.0625f,    // Gyro yaw P
                0);//0.005625f); // Gyro yaw I

void setup(void)
{
    h.init(new hf::Teensy32(), &rc, &mixer, &stabilizer);
}

void loop(void)
{
    h.update();
}
