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
#include "boards/realboards/arduino/butterfly_piggyback.hpp"
#include "sensors/usfs.hpp"
#include "openloops/receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "actuators/mixers/quadxcf.hpp"
#include "motors/standard.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/level.hpp"

#include "openloops/receivers/mock.hpp"
#include "motors/mock.hpp"

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 8.58f;
static const uint8_t MOTOR_PINS[4] = {5, 8 , 9, 11};

static hf::UsfsGyro gyro;
static hf::UsfsQuat quat;
static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);  
static hf::RatePid ratePid = hf::RatePid( 0.05f, 0.00f, 0.00f, 0.10f, 0.01f); 
static hf::LevelPid levelPid = hf::LevelPid(0.20f);
static hf::Butterfly board;

static hf::StandardMotor motors = hf::StandardMotor(MOTOR_PINS, 4);
static hf::MixerQuadXCF mixer(&motors);

// hf::MockReceiver receiver; 
// hf::MockMotor motors;

static hf::Hackflight h(&board, &receiver, &mixer);

void setup(void)
{
    // Add gyro, quaternion sensors
    h.addSensor(&gyro);
    h.addSensor(&quat);

    // Add rate and level PID controllers
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);

    // Initialize Hackflight firmware
    h.begin();
}

void loop(void)
{
    h.update();
}
