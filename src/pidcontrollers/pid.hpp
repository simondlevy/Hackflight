/*
   General PID controller support

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

#include "filters.hpp"
#include "datatypes.hpp"

namespace hf {

    class Pid {

        private: 

            // PID constants
            float _Kp = 0;
            float _Ki = 0;
            float _Kd = 0;

            // Accumulated values
            float _lastError   = 0;
            float _errorI      = 0;
            float _deltaError1 = 0;
            float _deltaError2 = 0;

            // For deltaT-based controllers
            float _previousTime = 0;
     
            // Prevents integral windup
            float _windupMax = 0;

        public:

            void init(const float Kp, const float Ki, const float Kd, const float windupMax=0.0) 
            {
                // Set constants
                _Kp = Kp;
                _Ki = Ki;
                _Kd = Kd;
                _windupMax = windupMax;

                // Initialize error integral, previous value
                reset();
            }

            // Version 1: ignore time
            float compute(float target, float actual)
            {
                // Compute error as scaled target minus actual
                float error = target - actual;

                // Compute P term
                float pterm = error * _Kp;

                // Compute I term
                float iterm = 0;
                if (_Ki > 0) { // optimization
                    _errorI = Filter::constrainAbs(_errorI + error, _windupMax); // avoid integral windup
                    iterm =  _errorI * _Ki;
                }

                // Compute D term
                float dterm = 0;
                if (_Kd > 0) { // optimization
                    float deltaError = error - _lastError;
                    dterm = (_deltaError1 + _deltaError2 + deltaError) * _Kd; 
                    _deltaError2 = _deltaError1;
                    _deltaError1 = deltaError;
                    _lastError = error;
                }

                return pterm + iterm + dterm;
            }

            // Version 2: use time
            float compute(float target, float actual, float currentTime)
            {
                // Don't do anything until we have a positive deltaT
                float deltaT = currentTime - _previousTime;
                _previousTime = currentTime;
                if (deltaT == currentTime) return 0;

                // Compute error as scaled target minus actual
                float error = target - actual;

                // Compute P term
                float pterm = error * _Kp;

                // Compute I term
                _errorI = Filter::constrainAbs(_errorI + error * deltaT, _windupMax);
                float iterm = _errorI * _Ki;

                // Compute D term
                float deltaError = (error - _lastError) / deltaT;
                float dterm = deltaError * _Kd;
                _lastError = error;

                return pterm + iterm + dterm;
            }

            void updateReceiver(demands_t & demands, bool throttleIsDown)
            {
                (void)demands; 

                // When landed, reset integral component of PID
                if (throttleIsDown) {
                    reset();
                }
            }

            void reset(void)
            {
                _errorI = 0;
                _lastError = 0;
                _previousTime = 0;
            }

    };  // class Pid

} // namespace hf
