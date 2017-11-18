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

                // Euler angles
                angles[0] = 0;
                angles[1] = 0;
                angles[2] = 0;
            }

            bool skipArming(void)
            {
                return true;
            }

            void getImu(float eulerAnglesRadians[3], float gyroRadiansPerSecond[3])
            {
                eulerAnglesRadians[0] = angles[0];
                eulerAnglesRadians[1] = angles[1];
                eulerAnglesRadians[2] = angles[2];

                gyroRadiansPerSecond[0] =  gyro[0];
                gyroRadiansPerSecond[1] = -gyro[1];
                gyroRadiansPerSecond[2] = -gyro[2];
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
                return baroPressurePascals;
            }

            virtual void extrasImuGetAccel(float accelGs[3])
            {
                for (uint8_t k = 0; k<3; ++k) {
                    accelGs[k] = 1.f; // XXX
                }
            }

            // This method is called by your simulator -----------------------------------------------------------------

            void updatePhysics(float rollSpeed, float pitchSpeed, float yawSpeed, float verticalPosition, float deltaSeconds)
            {
                // Track time
                micros += 1e6 * deltaSeconds;

                // Integrate Euler angle velocities to get Euler angles
                angles[0] += rollSpeed  * deltaSeconds;
                angles[1] -= pitchSpeed * deltaSeconds;
                angles[2] += yawSpeed   * deltaSeconds;

                // Compute pitch, roll, yaw first derivative to simulate gyro
                for (int k=0; k<3; ++k) {
                    gyro[k] = (angles[k] - anglesPrev[k]) / deltaSeconds;
                    anglesPrev[k] = angles[k];
                }

                // Convert vehicle's Z coordinate in meters to barometric pressure in Pascals (millibars)
                // At low altitudes above the sea level, the pressure decreases by about 1200 Pa for every 100 meters
                // (See https://en.wikipedia.org/wiki/Atmospheric_pressure#Altitude_variation)
                baroPressurePascals = 1000 * (101.325 - 1.2 * verticalPosition / 100);
            }

            // Public state variables
            float motors[4];
            float angles[3];

        private:

            // Private state variables
            uint64_t micros;
            float gyro[3];
            float anglesPrev[3];
            float baroPressurePascals;

    }; // class SimBoard

} // namespace hf
