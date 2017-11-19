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

        // private state variables
        private:

            // Loop time, set by simulator, then returned to main firmware via init(Config &)
            uint32_t imuLoopMicros;

            // Private state variables
            uint64_t micros;
            float accel[3];      // Gs
            float gyro[3];       // radians per second
            float baroPressure;  // Pascals (milllibars)

        // public state variables
        public:
            float motors[4];
            float angles[3];

        // public methods called by simulator
        public:

            SimBoard(uint32_t imuLoopTimeMicros)
            {
                // Store IMU loop period for when Hackflight calls Board::init(Config &)
                imuLoopMicros = imuLoopTimeMicros;

                // Initialize state variables

                for (uint8_t k=0; k<3; ++k) {
                    angles[k] = 0;
                    gyro[k] = 0;
                    accel[k] = 0;
                }

                for (uint8_t k=0; k<4; ++k) {
                    motors[k] = 0;
                }

                baroPressure = 0;
                micros = 0;
             }

            void updatePhysics(float angularVelocity[3], float altitude, float deltaSeconds)
            {
                // Track time
                micros += 1e6 * deltaSeconds;

                // Update state
                for (int k=0; k<3; ++k) {
                    angles[k] += ((k==1) ? -1 : +1) * angularVelocity[k] * deltaSeconds; // negate pitch
                    gyro[k] = angularVelocity[k];
                }

                /*
                // Estimate G forces on accelerometer using Equations 2, 6-8 in
                // https://www.nxp.com/docs/en/application-note/AN3461.pdf
                float phi   = angles[0]; // roll
                float theta = angles[1]; // pitch
                accel[0] = accelZ * -sin(theta);              // accel X   
                accel[1] = accelZ *  cos(theta) * sin(phi);   // accel Y   
                accel[2] = accelZ *  cos(theta) * cos(phi);   // accel Z   
                */

                // Convert vehicle's Z coordinate in meters to barometric pressure in Pascals (millibars)
                // At low altitudes above the sea level, the pressure decreases by about 1200 Pa for every 100 meters
                // (See https://en.wikipedia.org/wiki/Atmospheric_pressure#Altitude_variation)
                baroPressure = 1000 * (101.325 - 1.2 * altitude / 100);
            }


        // public methods called by Hackflight
        public:

            void init(Config& config)
            {
                // Loop timing overrides
                config.loop.imuLoopMicro = imuLoopMicros;

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

   }; // class SimBoard

} // namespace hf
