/*
   ThreeDFlyDSMX.ino : Hackflight sketch for Ladybug Flight Controller with 3DFly frame and Spektrum DSMX receiver

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

#include "boards/ladybug.hpp"

#include "receivers/serial/arduino_dsmx.hpp"

hf::Hackflight h;

hf::DSMX_Receiver rc = hf::DSMX_Receiver(
        .005f,  // roll trim
        .01f,  // pitch trim
        0.f);   // yaw trim

hf::Stabilizer stabilizer = hf::Stabilizer(
                0.20f,      // Level P
                0.225f,     // Gyro cyclic P
                0.001875f,  // Gyro cyclic I
                0.375f,     // Gyro cyclic D
                1.0625f,    // Gyro yaw P
                0.005625f); // Gyro yaw I

void setup(void)
{
    // Initialize Hackflight firmware
    h.init(new hf::Ladybug(), &rc, &stabilizer);
}

void loop(void)
{
    h.update();
}
