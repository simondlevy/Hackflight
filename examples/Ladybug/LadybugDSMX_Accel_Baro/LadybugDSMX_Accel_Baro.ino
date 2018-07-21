/*
   LadybugFlyDSMX_Accel_Baro.ino : Hackflight sketch for Ladybug Flight Controller with 
   Spektrum DSMX receiver and accelerometer/barometer access.

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
#include "boards/ladybug.hpp"
#include "receivers/serial/arduino_dsmx.hpp"
#include "mixers/quadx.hpp"
#include "sensors/accelerometer.hpp"
#include "sensors/barometer.hpp"

constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

hf::Hackflight h;

hf::DSMX_Receiver rc = hf::DSMX_Receiver(
        CHANNEL_MAP,
        .005f,  // roll trim
        .01f,  // pitch trim
        0.f);   // yaw trim

hf::MixerQuadX mixer;

hf::Stabilizer stabilizer = hf::Stabilizer(
                0.20f,      // Level P
                0.225f,     // Gyro cyclic P
                0.001875f,  // Gyro cyclic I
                0.375f,     // Gyro cyclic D
                1.0625f,    // Gyro yaw P
                0.005625f); // Gyro yaw I

hf::Accelerometer accel;

hf::Barometer baro;

void setup(void)
{
    // Create a Ladybug board object
    hf::Ladybug * board = new hf::Ladybug();

    // Initialize Hackflight firmware
    h.init(board, &rc, &mixer, &stabilizer);

    // Add the accelerometer and barometer to the sensors that will be acquired from this board
    h.addSensor(&accel, board);
    h.addSensor(&baro, board);
}

void loop(void)
{
    h.update();
}
