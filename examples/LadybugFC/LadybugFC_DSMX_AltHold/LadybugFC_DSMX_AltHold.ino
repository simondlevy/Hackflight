/*
   Hackflight sketch for Ladybug Flight Controller with Spektrum DSMX receiver and VL53L1X distance sensor, providing altitude-hold

   Copyright (c) 2019 Simon D. Levy

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
#include "boards/realboards/arduino/ladybugfc.hpp"
#include "receivers/arduino/dsmx_serial1.hpp"
#include "sensors/rangefinders/vl53l1x.hpp"
#include "pidcontrollers/level.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/althold.hpp"
#include "mixers/quadxcf.hpp"

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 1.0f;

hf::Hackflight h;

hf::DSMX_Receiver_Serial1 rc = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);

hf::MixerQuadXCF mixer;

hf::RatePid ratePid = hf::RatePid(
        0.225f,     // Gyro pitch/roll P
        0.001875f,  // Gyro pitch/roll I
        0.375f,     // Gyro pitch/roll D
        1.0625f,    // Gyro yaw P
        0.005625f); // Gyro yaw I

hf::LevelPid levelPid = hf::LevelPid(0.20f);

hf::AltitudeHoldPid altholdPid = hf::AltitudeHoldPid(
        1.00f,   // Altitude Hold P
        0.15f,   // Altitude Hold Velocity P
        0.01f,   // Altitude Hold Velocity I
        0.05f);  // Altitude Hold Velocity D

hf::VL53L1X_Rangefinder rangefinder;

void setup(void)
{
    // Add some "software trim" to the receiver
    rc.setTrimRoll(+.2);
    rc.setTrimPitch(+.3);

    // Initialize Hackflight firmware
    h.init(new hf::LadybugFC(), &hf::ladybugIMU, &rc, &mixer, hf::ladybugFcMotors);

    // Add rangefinder sensor
    rangefinder.begin();
    h.addSensor(&rangefinder);

    // Add PID controllers
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);
    h.addPidController(&altholdPid, 1);
}

void loop(void)
{
    h.update();

}
