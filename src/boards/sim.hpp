/*
   sim.hpp: Hackflight Board class implementation for flight simulators

   Copyright (C) Simon D. Levy 2017

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

#pragma once

#include <board.hpp>
#include <debug.hpp>

namespace hf {

    class SimBoard : public Board {

        public:

            // These methods are called by Hackflight ----------------------------------------------------

            void init(Config& config)
            {
                // Loop timing overrides
                config.loop.imuLoopMicro = 8333;    // approx. simulation period

                // State variables
                for (uint8_t k=0; k<3; ++k) {
                    angles[k] = 0;
                    gyro[k] = 0;
                    accel[k] = 0;
                }
            }

            bool skipArming(void)
            {
                return true;
            }

            void getImu(float eulerAnglesRadians[3], float gyroRadiansPerSecond[3])
            {
                for (uint8_t k=0; k<3; ++k) {
                    eulerAnglesRadians[k] = angles[k];
                    gyroRadiansPerSecond[k] = -gyro[k];
                }
            }

            uint64_t getMicros()
            {
                return micros;
            }

            void writeMotor(uint8_t index, float value)
            {
                motors[index] = value;
            }

            void delayMilliseconds(uint32_t msec)
            {
            }

            virtual bool extrasHaveBaro(void)
            {
                return true;
            }

            virtual float extrasGetBaroPressure(void)
            {
                return baroPressure;
            }

            virtual void extrasImuGetAccel(float accelGs[3])
            {
                for (uint8_t k = 0; k<3; ++k) {
                    accelGs[k] = 1.f; // XXX
                }
            }

            // This method is called by your simulator -----------------------------------------------------------------

            void updatePhysics(float angularVelocity[3], float altitude, float verticalAcceleration, float deltaSeconds)
            {
                // Track time
                micros += 1e6 * deltaSeconds;

                // Update state
                for (int k=0; k<3; ++k) {
                    angles[k] += angularVelocity[k] * deltaSeconds; // XXX Does pitch need to be negated first?
                    gyro[k] = (angles[k] - anglesPrev[k]) / deltaSeconds;
                    anglesPrev[k] = angles[k];
                }

                // Negate vertical acceleration to get correct direction for G force
                float accZ = -verticalAcceleration;

                // Estimate G forces on accelerometer using Equations 2, 6-8 in
                // https://www.nxp.com/docs/en/application-note/AN3461.pdf
                float phi   = angles[0]; // roll
                float theta = angles[1]; // pitch
                accel[0] = accZ * sin(theta);              // accel X   
                accel[1] = accZ * cos(theta) * sin(phi);   // accel Y   
                accel[2] = accZ * cos(theta) * cos(phi);   // accel Z   

                dprintf("%+2.2f    %+2.2f    %+2.2f\n", accel[0], accel[1], accel[2]);

                // Convert vehicle's Z coordinate in meters to barometric pressure in Pascals (millibars)
                // At low altitudes above the sea level, the pressure decreases by about 1200 Pa for every 100 meters
                // (See https://en.wikipedia.org/wiki/Atmospheric_pressure#Altitude_variation)
                baroPressure = 1000 * (101.325 - 1.2 * altitude / 100);
            }

            // Public state variables
            float motors[4];
            float angles[3];

        private:

            // Private state variables
            uint64_t micros;
            float accel[3];      // Gs
            float gyro[3];       // radians per second
            float anglesPrev[3]; // radians
            float baroPressure;  // Pascals (milllibars)

    }; // class SimBoard

} // namespace hf
