/*
   sim.hpp: Hackflight Board class implementation for flight simulators

   Simulates quadcopter physics (vehicle state)

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
#include <datatypes.hpp>

#include <time.h>
#include <stdio.h>

namespace hf {

    class SimBoard : public Board {

        private:

            // Constants ------------------------------------------

            // Some true constants!
            const float GRAVITY = 9.80665;

            // Scales up spin rate in [0,1] to thrust in Newtons (substitutes for mass, torque, etc.)
            const float THRUST_SCALE = 5.5;

            // Approxmiate zero for liftoff
            const float NOISE_FLOOR = 0.2;

            // Controls "snappiness" of response
            const float MOTOR_EXPONENT = 3;

            // Private state variables ----------------------------
            float _verticalSpeedPrev; // meters per second
            float _eulerAngles[3];
            float _gyroRates[3];        
            float _translationRates[3]; // local (body) frame
            float _position[3];
            float _motors[4];         // arbitrary in [0,1]
            bool  _flying;
            float _secondsPrev;

            // Gets CPU time in seconds
            void cputime(struct timespec * tv);

        public:

            // accessor available to simulators -----------------------------------------------

            void simGetVehicleState(float gyroRates[3], float translationRates[3], float motors[4])
            {
                memcpy(gyroRates, _gyroRates, 3*sizeof(float));
                memcpy(translationRates, _translationRates, 3*sizeof(float));
                memcpy(motors, _motors, 4*sizeof(float));
            }

            // methods called by Hackflight -------------------------------------------------

            // Init physics
            void init(void)
            {
                _secondsPrev = 0;
                memset(_motors, 0, 4*sizeof(float));
                memset(_eulerAngles, 0, 3*sizeof(float));
                memset(_gyroRates, 0, 3*sizeof(float));
                memset(_translationRates, 0, 3*sizeof(float));
                memset(_position, 0, 3*sizeof(float));
                _flying = false;
                _verticalSpeedPrev = 0;
            }

            // Sync physics update to gyro acquisition
            bool getGyroRates(float gyroRates[3])
            {
                // Compute body-frame roll, pitch, yaw velocities based on differences between motors
                _gyroRates[0] = motorsToAngularVelocity(2, 3, 0, 1);
                _gyroRates[1] = motorsToAngularVelocity(1, 3, 0, 2); 
                _gyroRates[2] = motorsToAngularVelocity(1, 2, 0, 3); 

                // Overall thrust vector, scaled by arbitrary constant for realism
                float thrust = THRUST_SCALE * (_motors[0] + _motors[1] + _motors[2] + _motors[3]);

                // Rename Euler angles to familiar Greek-letter variables
                float phi   = _eulerAngles[0];
                float theta = _eulerAngles[1];

                // Overall vertical force = thrust - gravity
                float lift = thrust - GRAVITY;

                // Compute delta seconds
                float secondsCurr = seconds();
                float deltaSeconds = secondsCurr - _secondsPrev;
                _secondsPrev = secondsCurr;

                // Once there's enough lift, we're flying
                if (lift > NOISE_FLOOR) {
                    _flying = true;
                }

                if (_flying) {

                    // Integrate vertical force to get vertical speed
                    _translationRates[2] += (lift * deltaSeconds);

                    // To get forward and lateral speeds, integrate thrust along vehicle coordinates
                    _translationRates[0] += thrust * deltaSeconds * sin(theta);
                    _translationRates[1] += thrust * deltaSeconds * sin(phi);
                }

                // Integrate speed to get position 
                for (int8_t k=0; k<3; ++k) {
                    _position[k] += _translationRates[k] * deltaSeconds;
                }

                // Integrate gyro to get eulerAngles, negating pitch
                for (int k=0; k<3; ++k) {
                    _eulerAngles[k] += ((k==1) ? -1 : +1) * _gyroRates[k] * deltaSeconds; 
                }

                // Differentiate vertical speed to get vertical acceleration in meters per second, then convert to Gs.
                // Resting = 1G; freefall = 0; climbing = >1G
                _verticalSpeedPrev = _translationRates[2];

                memcpy(gyroRates, _gyroRates, 3*sizeof(float));

                return true;
            }

            bool getEulerAngles(float eulerAngles[3]) {

                memcpy(eulerAngles, _eulerAngles, 3*sizeof(float));

                return true;
            }

            uint32_t getMicroseconds()
            {
                return seconds() * 1000000;
            }

            void writeMotor(uint8_t index, float value)
            {
                _motors[index] = value;
            }

        private:

            float motorsToAngularVelocity(int a, int b, int c, int d)
            {
                float v = ((_motors[a] + _motors[b]) - (_motors[c] + _motors[d]));

                return (v<0 ? -1 : +1) * pow(fabs(v), MOTOR_EXPONENT);
            }

           float seconds()
            {
                struct timespec t;
                cputime(&t);
                return t.tv_sec + t.tv_nsec/1.e9;
            }

    }; // class SimBoard

} // namespace hf
