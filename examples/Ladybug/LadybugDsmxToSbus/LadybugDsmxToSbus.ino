/*
   HackflightLite sketch for Ladybug dev board with DSMX => SBUS pass-thru

   Additional libraries needed:

       https://github.com/simondlevy/DSMRX
       https://github.com/bolderflight/SBUS

       https://github.com/simondlevy/CrossPlatformDataBus

   Hardware support for SMT32L4:

       https://github.com/simondlevy/grumpyoldpizza

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

#include <Arduino.h>

//#include <DSMRX.h>

#include "hackflightlite.hpp"
#include "boards/realboards/arduino/ladybug.hpp"
#include "receivers/arduino/dsmx_serial2.hpp"
#include "rxproxies/sbus.hpp"

static const uint8_t CHANNELS = 8;

static constexpr uint8_t CHANNEL_MAP[6] = {0,1,2,3,4,5};

static constexpr float DEMAND_SCALE = 1.0f;

hf::HackflightLite h;

hf::DSMX_Receiver_Serial2 rc = hf::DSMX_Receiver_Serial2(CHANNEL_MAP, DEMAND_SCALE);  

hf::SbusProxy px;

void setup(void)
{
    Serial.begin(115000);
    rc.begin();

    //h.init(new hf::Ladybug(), &rc, &px);
}

void loop(void)
{
    //h.update();

    if (rc.gotNewFrame()) {
       rc.readRawvals(); 
       Serial.println(rc.rawvals[0]);
    }
}
