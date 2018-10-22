/*
   SBUSAltHold.ino : Hackflight sketch for Bonadrone flight controller with SBUS receiver
   that allows altitude hold flight mode

   Additional libraries needed:

       https://github.com/simondlevy/LSM6DSM
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SBUSRX
       https://github.com/simondlevy/VL53L1X

   Hardware support for Bonadrone flight controller:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (c) 2018 Juan Gallostra & Simon D. Levy

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
#include <VL53L1X.h>

#include "hackflight.hpp"
#include "boards/bonadrone.hpp"
#include "receivers/sbus.hpp"
#include "mixers/quadx.hpp"

#include "sensors/rangefinder.hpp"

#include "pidcontrollers/level.hpp"
#include "pidcontrollers/althold.hpp"

class VL53L1X_Rangefinder : public hf::Rangefinder {

    private:

        VL53L1X _distanceSensor;

    protected:

        virtual bool distanceAvailable(float & distance) override
        {
            if (_distanceSensor.newDataReady()) {
                distance = _distanceSensor.getDistance() / 1000.f; // mm => m
                return true;
            }
            return false;
        }

    public:

        void begin(void)
        {
            _distanceSensor.begin();
        }
}; // class VL53L1X_Rangefinder 

// Change this as needed
#define SBUS_SERIAL Serial1

static constexpr uint8_t CHANNEL_MAP[6] = {2,0,1,3,5,4};

hf::Hackflight h;

hf::SBUS_Receiver rc = hf::SBUS_Receiver(CHANNEL_MAP, SERIAL_SBUS, &SBUS_SERIAL);

hf::MixerQuadX mixer;

VL53L1X_Rangefinder rangefinder;

hf::Rate ratePid = hf::Rate(
        0.10f,  // Gyro Roll/Pitch P
        0.01f,  // Gyro Roll/Pitch I
        0.05f,  // Gyro Roll/Pitch D
        0.10f,  // Gyro yaw P
        0.01f,  // Gyro yaw I
        5.00f); // Demands to rate

hf::Level level = hf::Level(
        0.25f,   // Roll Level P
        0.25f);  // Pitch Level P

hf::AltitudeHold althold = hf::AltitudeHold(
        1.00f,   // Altitude Hold P
        0.15f,   // Altitude Hold Velocity P
        0.01f,   // Altitude Hold Velocity I
        0.05f);  // Altitude Hold Velocity D

void setup(void)
{
    // begin the serial port for the ESP32
    Serial4.begin(115200);

    // Trim receiver via software
    rc.setTrimRoll(-0.0012494f);
    rc.setTrimPitch(-0.0058769f);
    rc.setTrimYaw(-0.0192190f);

    // 0 means the controller will always be active, but by changing
    // that number it can be linked to a different aux state
    h.addPidController(&althold, 2);
    h.addPidController(&level, 0);

    h.init(new hf::BonadroneMultiShot(), &rc, &mixer, &ratePid);
    
    // Add rangefinder sensor
    rangefinder.begin();
    h.addSensor(&rangefinder);
    
}

void loop(void)
{
    h.update();
}
