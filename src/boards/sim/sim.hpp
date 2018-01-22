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
            const float SEALEVEL = 1013.25; // millibars

            // Scales up spin rate in [0,1] to thrust in Newtons (substitutes for mass, torque, etc.)
            const float THRUST_SCALE = 5.5;

            // Approxmiate zero for liftoff
            const float NOISE_FLOOR = 0.2;

            // Controls "snappiness" of response
            const float VELOCITY_ROTATE_SCALE    = 1.75;

            // Private state variables ----------------------------
            float _linearSpeeds[3];   // meters per second forward, lateral, vertical
            float _verticalSpeedPrev; // meters per second
            float _motors[4];         // arbitrary in [0,1]

            vehicle_state_t _vehicleState;

            float _baroPressure;      // millibars
            bool  _flying;
            float _accel[3];          // Gs
            float _secondsPrev;

            bool drifted;

            // Gets CPU time in seconds
            void cputime(struct timespec * tv);

        public:

            // accessors available to simulators -----------------------------------------------

            void simGetEulerAngles(float angles[3])
            {
                for (uint8_t k=0; k<3; ++k) {
                    angles[k] = _vehicleState.orientation[k].value;
                }
            }

            void simGetGyro(float gyro[3])
            {
                for (uint8_t k=0; k<3; ++k) {
                    gyro[k] = _vehicleState.orientation[k].deriv;
                }
            }

            void simGetLinearSpeeds(float speeds[3])
            {
                for (uint8_t k=0; k<3; ++k) {
                    speeds[k] = _linearSpeeds[k];
                }
            }

            void simGetMotors(float motors[3])
            {
                for (uint8_t k=0; k<4; ++k) {
                    motors[k] = _motors[k];
                }
            }

            float simGetAltitude(void)
            {
                return _vehicleState.position[2].value;
            }

            bool simIsFlying(void)
            {
                return _flying;
            }

            // methods called by Hackflight -------------------------------------------------

            void init(void)
            {
                _secondsPrev = 0;
                initPhysics();

                drifted = false;
            }

            void getState(vehicle_state_t * vehicleState)
            {
                memcpy(vehicleState, &_vehicleState, sizeof(vehicle_state_t));

                // Sync physics update to IMU acquisition by Hackflight
                updatePhysics();
            }

            uint32_t getMicroseconds()
            {
                return seconds() * 1000000;
            }

            void writeMotor(uint8_t index, float value)
            {
                _motors[index] = value;
            }

            virtual bool extrasHaveBaro(void)
            {
                return true;
            }

            virtual float extrasGetBaroPressure(void)
            {
                return _baroPressure;
            }

            virtual void extrasImuGetAccel(float accelGs[3])
            {
                for (uint8_t k=0; k<3; ++k) {
                    accelGs[k] = _accel[k];
                }
            }

            // private methods
        private:

            void initPhysics(void)
            {
                for (uint8_t k=0; k<3; ++k) {
                    _vehicleState.orientation[k].value = 0;
                    _vehicleState.orientation[k].deriv = 0;
                    _accel[k] = 0;
                    _linearSpeeds[k] = 0;
                }

                for (uint8_t k=0; k<4; ++k) {
                    _motors[k] = 0;
                }

                _flying = false;
                _vehicleState.position[2].value = 0;
                _baroPressure = SEALEVEL;
                _verticalSpeedPrev = 0;
            }

            void updatePhysics(void)
            {
                // Compute body-frame roll, pitch, yaw velocities based on differences between motors
                _vehicleState.orientation[0].deriv = motorsToAngularVelocity(2, 3, 0, 1);
                _vehicleState.orientation[1].deriv = motorsToAngularVelocity(1, 3, 0, 2); 
                _vehicleState.orientation[2].deriv = motorsToAngularVelocity(1, 2, 0, 3); 

                // Overall thrust vector, scaled by arbitrary constant for realism
                float thrust = THRUST_SCALE * (_motors[0] + _motors[1] + _motors[2] + _motors[3]);

                // Rename Euler angles to familiar Greek-letter variables
                float phi   = _vehicleState.orientation[0].value;
                float theta = _vehicleState.orientation[1].value;
                float psi   = _vehicleState.orientation[2].value;

                // Overall vertical force = thrust - gravity
                float lift = cos(phi)*cos(theta)*thrust - GRAVITY;

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
                    _linearSpeeds[2] += (lift * deltaSeconds);

                    // To get forward and lateral speeds, integrate thrust along vehicle coordinates
                    _linearSpeeds[0] += thrust * deltaSeconds * sin(theta);
                    _linearSpeeds[1] += thrust * deltaSeconds * sin(phi);

                    // Add some drift
                    if (!drifted) {
                        //_linearSpeeds[1] += 0.5;
                        drifted = true;
                    }
                }

                // Integrate vertical speed to get altitude
                _vehicleState.position[2].value += _linearSpeeds[2] * deltaSeconds;

                // Reset everything if we hit the ground
                if (_vehicleState.position[2].value < 0) {
                    initPhysics();
                }

                // Update state
                for (int k=0; k<3; ++k) {
                    _vehicleState.orientation[k].value += ((k==1) ? -1 : +1) * _vehicleState.orientation[k].deriv * deltaSeconds; // negate pitch
                }

                // Differentiate vertical speed to get vertical acceleration in meters per second, then convert to Gs.
                // Resting = 1G; freefall = 0; climbing = >1G
                float g = (_linearSpeeds[2]-_verticalSpeedPrev)/deltaSeconds/ GRAVITY + 1;
                _verticalSpeedPrev = _linearSpeeds[2];

                // Estimate G forces on accelerometer using Equations 2, 6-8 in
                // https://www.nxp.com/docs/en/application-note/AN3461.pdf
                _accel[0] = g * -sin(theta);              // X   
                _accel[1] = g *  cos(theta) * sin(phi);   // Y   
                _accel[2] = g *  cos(theta) * cos(phi);   // Z   

                // Convert vehicle's Z coordinate in meters to barometric pressure in millibars
                // At low altitudes above the sea level, the pressure decreases
                // by about 12 millbars (1.2kPascals) for every 100 meters (See
                // https://en.wikipedia.org/wiki/Atmospheric_pressure#Altitude_variation)
                _baroPressure = SEALEVEL - 12 * _vehicleState.position[2].value / 100;
            }

            float motorsToAngularVelocity(int a, int b, int c, int d)
            {
                return VELOCITY_ROTATE_SCALE * ((_motors[a] + _motors[b]) - (_motors[c] + _motors[d]));
            }

           float seconds()
            {
                struct timespec t;
                cputime(&t);
                return t.tv_sec + t.tv_nsec/1.e9;
            }

    }; // class SimBoard

} // namespace hf
