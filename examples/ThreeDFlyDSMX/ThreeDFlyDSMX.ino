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
#include "models/3dfly.hpp"
#include "receivers/rc/serial/dsmx.hpp"

hf::Hackflight h;

hf::ThreeDFly model;

hf::DSMX_Receiver rc;

// Additional PID controllers
//#include <extras/altitude_hold.hpp>
//hf::AltitudeHold altitudeHold = hf::AltitudeHold(0.1f, 0.02f, 1.7f);

void setup(void)
{
    // Add altithude-hold feature to Hackflight firmware
    //h.addPidController(&altitudeHold);

    // Initialize Hackflight firmware
    h.init(new hf::Ladybug(), &rc, &model);
}

void loop(void)
{
    h.update();
}
