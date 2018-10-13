/*
   DSMX_Loiter.ino : Hackflight sketch for Ladybug Flight Controller with Spektrum DSMX receiver,
                     VL53L1X distance sensor, and PMW3901 optical-flow sensor

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
#include <VL53L1X.h>
#include <PMW3901.h>

#include "hackflight.hpp"
#include "boards/ladybug.hpp"
#include "receivers/dsmx.hpp"
#include "sensors/rangefinder.hpp"
#include "sensors/opticalflow.hpp"
#include "pidcontrollers/level.hpp"
#include "pidcontrollers/loiter.hpp"
#include "mixers/quadx.hpp"

constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

hf::Hackflight h;

hf::DSMX_Receiver rc = hf::DSMX_Receiver(CHANNEL_MAP);

hf::MixerQuadX mixer;

hf::Rate ratePid = hf::Rate(
        0.225f,     // Gyro pitch/roll P
        0.001875f,  // Gyro pitch/roll I
        0.375f,     // Gyro pitch/roll D
        1.0625f,    // Gyro yaw P
        0.005625f); // Gyro yaw I

hf::Level level = hf::Level(0.20f);

hf::Loiter loiter = hf::Loiter(
	0.40f,  // Altitude P
    0.2f,   // Altitude D
    0.f,    // Cyclic P
    1.0f,   // throttleScale
    0.04f); // minAltitude


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


class PMW3901_OpticalFlow : public hf::OpticalFlow {

    private:

        // Using digital pin 10 for chip select
        PMW3901 _flowSensor = PMW3901(10);


    protected:

        virtual void getFlow(float flow[2]) override
        {
            int16_t deltaX=0, deltaY=0;
            _flowSensor.readMotionCount(&deltaX, &deltaY);
            flow[0] = (float)deltaX;
            flow[1] = (float)deltaY;
        }

    public:

        void begin(void)
        {
            if (!_flowSensor.begin()) {
                while (true) {
                    Serial.println("Initialization of the flow sensor failed");
                    delay(500);
                }
            }

        }

};  // class PMW3901_OpticalFlow 

VL53L1X_Rangefinder rangefinder;

PMW3901_OpticalFlow opticalFlow;

void setup(void)
{
    // Add some "software trim" to the receiver
    rc.setTrimRoll(+.2);
    rc.setTrimPitch(+.1);

    // Initialize Hackflight firmware
    // We're using an older ladybug with LED on pin A1
    h.init(new hf::Ladybug(), &rc, &mixer, &ratePid);

    // Add rangefinder sensor
    rangefinder.begin();
    h.addSensor(&rangefinder);

    // Add optical-flow sensor
    opticalFlow.begin();
    h.addSensor(&opticalFlow);

    // Add Level PID for aux switch position 1
    h.addPidController(&level, 1);

    // Add Loiter PID controller for aux switch position 2
    h.addPidController(&loiter, 2);
}

void loop(void)
{
    h.update();

}
