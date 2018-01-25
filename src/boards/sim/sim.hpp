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
            const float VELOCITY_ROTATE_SCALE    = 1.75;

            // Private state variables ----------------------------
            float _verticalSpeedPrev; // meters per second
            float _motors[4];         // arbitrary in [0,1]

            vehicle_state_t _vehicleState;

            //AltitudeEstimator   altitudeEstimator;
            //Timer altitudeTimer = Timer(40);

            //float _baroPressure;      // millibars
            bool  _flying;
            float _secondsPrev;

            // Gets CPU time in seconds
            void cputime(struct timespec * tv);

        public:

            // accessor available to simulators -----------------------------------------------

            void simGetVehicleState(vehicle_state_t * vehicleState, float motors[4], bool * flying)
            {
                memcpy(vehicleState, &_vehicleState, sizeof(vehicle_state_t));

                memcpy(motors, _motors, 4*sizeof(float));

                *flying = _flying;
            }

            // methods called by Hackflight -------------------------------------------------

            void init(void)
            {
                _secondsPrev = 0;
                initPhysics();

                // Initialize the atitude estimator
                //altitudeEstimator.init(this);
            }

            void getState(vehicle_state_t & vehicleState)
            {
                memcpy(&vehicleState.pose, &_vehicleState.pose, sizeof(pose_t));

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

        private:

            void initPhysics(void)
            {
                memset(&_vehicleState, 0, sizeof(_vehicleState));

                for (uint8_t k=0; k<4; ++k) {
                    _motors[k] = 0;
                }

                _flying = false;
                _vehicleState.pose.position[2].value = 0;
                _verticalSpeedPrev = 0;
            }

            void updatePhysics(void)
            {
                // Compute body-frame roll, pitch, yaw velocities based on differences between motors
                _vehicleState.pose.orientation[0].deriv = motorsToAngularVelocity(2, 3, 0, 1);
                _vehicleState.pose.orientation[1].deriv = motorsToAngularVelocity(1, 3, 0, 2); 
                _vehicleState.pose.orientation[2].deriv = motorsToAngularVelocity(1, 2, 0, 3); 

                // Overall thrust vector, scaled by arbitrary constant for realism
                float thrust = THRUST_SCALE * (_motors[0] + _motors[1] + _motors[2] + _motors[3]);

                // Rename Euler angles to familiar Greek-letter variables
                float phi   = _vehicleState.pose.orientation[0].value;
                float theta = _vehicleState.pose.orientation[1].value;
                float psi   = _vehicleState.pose.orientation[2].value;

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
                    _vehicleState.pose.position[2].deriv += (lift * deltaSeconds);

                    // To get forward and lateral speeds, integrate thrust along vehicle coordinates
                    _vehicleState.pose.position[0].deriv += thrust * deltaSeconds * sin(theta);
                    _vehicleState.pose.position[1].deriv += thrust * deltaSeconds * sin(phi);
                }

                // Integrate vertical speed to get altitude
                _vehicleState.pose.position[2].value += _vehicleState.pose.position[2].deriv * deltaSeconds;

                // Reset everything if we hit the ground
                if (_vehicleState.pose.position[2].value < 0) {
                    initPhysics();
                }

                // Update state
                for (int k=0; k<3; ++k) {
                    _vehicleState.pose.orientation[k].value += ((k==1) ? -1 : +1) * _vehicleState.pose.orientation[k].deriv * deltaSeconds; // negate pitch
                }

                // Differentiate vertical speed to get vertical acceleration in meters per second, then convert to Gs.
                // Resting = 1G; freefall = 0; climbing = >1G
                float g = (_vehicleState.pose.position[2].deriv - _verticalSpeedPrev)/deltaSeconds/ GRAVITY + 1;
                _verticalSpeedPrev = _vehicleState.pose.position[2].deriv;
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
