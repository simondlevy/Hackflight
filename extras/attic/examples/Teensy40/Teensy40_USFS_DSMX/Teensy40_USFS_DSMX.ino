/*
   Hackflight sketch for Teensy 4.0 board with Ultimate Sensor Fusion Solution IMU and DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

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

#include <i2c_t3.h>

#include "hackflight.hpp"
#include "boards/realboards/arduino/teensy40.hpp"
#include "imus/usfs_inverted.hpp"
#include "receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "motors/mock.hpp"
#include "actuators/mixers/quadxcf.hpp"

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 8.58f;

hf::Hackflight h;

hf::USFS_Inverted imu;

hf::DSMX_Receiver_Serial1 rc = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);  

hf::MixerQuadXCF mixer;

hf::MockMotor motors;

void setup(void)
{
    hf::ArduinoBoard::powerPins(21, 22);

    delay(100);

    Wire.begin();

    delay(100);

    h.init(new hf::Teensy40(), &imu, &rc, &mixer, &motors);
}

void loop()
{  
    h.update();
}
