/*
   Common support for rate-based pitch, roll, yaw PID controllers

   Supports yaw stabilization and acro mode

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

#include "receiver.hpp"
#include "filters.hpp"
#include "datatypes.hpp"

namespace hf {

    class RateBased {

        friend class Hackflight;

        private: 

        // Arbitrary constants
        static constexpr float WINDUP_MAX             = 6.0f;
        static constexpr float BIG_DEGREES_PER_SECOND = 40.0f; 

        // Converted to radians from degrees in constructor for efficiency
        float _bigAngularVelocity = 0;

        // PID constants set in init() method
        float _Kp = 0;
        float _Ki = 0;
        float _Kd = 0;

        // Accumulated values
        float _lastError   = 0;
        float _errorI      = 0;
        float _deltaError1 = 0;
        float _deltaError2 = 0;
 
        // Scale factor for stick demand
        float _demandScale = 0;

        public:

        void init(const float Kp, const float Ki, const float Kd, const float demandScale) 
        {
            // Set constants
            _Kp = Kp;
            _Ki = Ki;
            _Kd = Kd;
            _demandScale = demandScale;

            // Zero-out previous values for D term
            _lastError = 0;

            // Convert degree parameters to radians for use later
            _bigAngularVelocity = Filter::deg2rad(BIG_DEGREES_PER_SECOND);

            // Initialize error integral
            _errorI = 0;
        }

        float compute(float demand, float angularVelocity, float itermFactor)
        {
            // Compute error as scaled demand minus angular velocity
            float error = demand * _demandScale - angularVelocity;

            // Compute P term
            float pterm = error * _Kp;

            // Avoid integral windup
            _errorI = Filter::constrainAbs(_errorI + error, WINDUP_MAX);

            // Reset integral on quick angular velocity change
            if (fabs(angularVelocity) > _bigAngularVelocity) {
                _errorI = 0;
            }

            // Compute I term
            float iterm =  _errorI * _Ki;

            // Compute D term
            float dterm = 0;
            if (_Kd > 0) { // optimization
                float deltaError = error - _lastError;
                float deltaErrorSum = _deltaError1 + _deltaError2 + deltaError;
                _deltaError2 = _deltaError1;
                _deltaError1 = deltaError;
                dterm = deltaErrorSum * _Kd; 
                _lastError = error;
            }

            return pterm + iterm + dterm;
        }

        void updateReceiver(demands_t & demands, bool throttleIsDown)
        {
            // When landed, reset integral component of PID
            if (throttleIsDown) {
                _errorI = 0; 
            }
        }

        void resetIntegral(void)
        {
            _errorI = 0;
        }

    };  // class RateBased

} // namespace hf
