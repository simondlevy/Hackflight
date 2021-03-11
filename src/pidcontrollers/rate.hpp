/*
   Angular-velocity-based PID controller for roll, pitch, yaw

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

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

#include "openloops/receiver.hpp"
#include "filters.hpp"
#include "state.hpp"
#include "demands.hpp"
#include "pidcontroller.hpp"

namespace hf {

    // Helper class for all three axes
    class _AngularVelocityPid : public Pid {

        private: 

            // Arbitrary constants
            static constexpr float BIG_DEGREES_PER_SECOND = 40.0f; 
            static constexpr float WINDUP_MAX = 6.0f;

            // Converted to radians from degrees in constructor for efficiency
            float _bigAngularVelocity = 0;

        public:

            void begin(const float Kp, const float Ki, const float Kd) 
            {
                Pid::begin(Kp, Ki, Kd, WINDUP_MAX);

                // Convert degree parameters to radians for use later
                _bigAngularVelocity = Filter::deg2rad(BIG_DEGREES_PER_SECOND);
            }

            float compute(float demand, float angularVelocity)
            {
                // Reset integral on quick angular velocity change
                if (fabs(angularVelocity) > _bigAngularVelocity) {
                    reset();
                }

                return Pid::compute(demand, angularVelocity);
            }

    };  // class _AngularVelocityPid

    class RatePid : public PidController {

        private: 

            // Aribtrary constants
            static constexpr float BIG_YAW_DEMAND = 0.1f;

            // Rate mode uses a rate controller for roll, pitch
            _AngularVelocityPid _rollPid;
            _AngularVelocityPid _pitchPid;
            _AngularVelocityPid _yawPid;

        public:

            RatePid(const float Kp, const float Ki, const float Kd, const float Kp_yaw, const float Ki_yaw) 
            {
                _rollPid.begin(Kp, Ki, Kd);
                _pitchPid.begin(Kp, Ki, Kd);
                _yawPid.begin(Kp_yaw, Ki_yaw, 0);
            }

            void modifyDemands(state_t * state, demands_t & demands)
            {
                demands.roll  = _rollPid.compute(demands.roll, state->x[STATE_DPHI]);

                // XXX Why do we have to negate pitch, yaw demands and state values?
                demands.pitch = _pitchPid.compute(-demands.pitch, -state->x[STATE_DTHETA]);
                demands.yaw   = _yawPid.compute(-demands.yaw, -state->x[STATE_DPSI]);

                // Prevent "yaw jump" during correction
                demands.yaw = Filter::constrainAbs(demands.yaw, 0.1 + fabs(demands.yaw));

                // Reset yaw integral on large yaw command
                if (fabs(demands.yaw) > BIG_YAW_DEMAND) {
                    _yawPid.reset();
                }
            }

            virtual void resetOnInactivity(bool inactive) override
            {
                // Check throttle-down for integral reset
                _rollPid.resetOnInactivity(inactive);
                _pitchPid.resetOnInactivity(inactive);
                _yawPid.resetOnInactivity(inactive);
            }

    };  // class RatePid

} // namespace hf
