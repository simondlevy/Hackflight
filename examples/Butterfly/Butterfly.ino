/*
   Hackflight sketch for Butterfly board with Ultimate Sensor Fusion Solution IMU and DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Hardware support for Butterfly flight controller:

       https://github.com/simondlevy/grumpyoldpizza

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
#include "boards/realboards/arduino/butterfly.hpp"
#include "imus/usfs.hpp"
// #include "receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "actuators/mixers/quadxcf.hpp"
// #include "motors/standard.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/level.hpp"

#include "receivers/mock.hpp"
#include "motors/mock.hpp"

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 8.58f;

hf::Hackflight h;
hf::USFS imu;
// hf::DSMX_Receiver_Serial1 rc = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);  
hf::MixerQuadXCF mixer;
hf::RatePid ratePid = hf::RatePid( 0.05f, 0.00f, 0.00f, 0.10f, 0.01f); 
hf::LevelPid levelPid = hf::LevelPid(0.20f);

// hf::StandardMotor motor1(5), motor2(8), motor3(9), motor4(11);
// hf::Motor * motors[4] = { &motor1, &motor2, &motor3, &motor4 };

hf::MockReceiver rc;
hf::MockMotor motors;

void setup(void)
{
    // Initialize Hackflight firmware
    h.init(new hf::Butterfly(), &imu, &rc, &mixer, &motors);

    // Add Rate and Level PID controllers
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);
}

void loop(void)
{
    h.update();
}
