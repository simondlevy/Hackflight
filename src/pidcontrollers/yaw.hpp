/*
   yaw.hpp : yaw PID controller

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
#include "pidcontroller.hpp"

namespace hf {

    class YawPid : public PidController {

        friend class Hackflight;

        private: 

            // Arbitrary constants
            const float BIG_YAW_DEMAND = 0.1f;

            // Arbitrary constants
            const float WINDUP_MAX             = 6.0f;
            const float BIG_DEGREES_PER_SECOND = 40.0f; 

            // Converted to radians from degrees in constructor for efficiency
            float _bigAngularVel = 0;

            // PID constants set in constructor
            float _P = 0; 
            float _I = 0;

            // Accumulated values
            float _lastError = 0;
            float _errorI    = 0;

            float computeITerm(float error, float rcCommand, float angularVel)
            {
                // Avoid integral windup
                _errorI = Filter::constrainAbs(_errorI + error, WINDUP_MAX);

                // Reset integral on quick gyro change or large gyroYawPid command
                if (fabs(angularVel) > _bigAngularVel) {
                    _errorI = 0;
                }

                return _errorI * _I;
            }

            float computePid(float PTerm, float ITerm, float DTerm, float angularVel)
            {
                PTerm = (PTerm * _demandsToRate - angularVel) * _P;

                return PTerm + ITerm + DTerm;
            }

        protected:

            float _demandsToRate = 0;

            void resetIntegral(void)
            {
                _errorI = 0;
            }

        public:

            YawPid(float P, float I, float demandsToRate = 1.0f) 
                : _P(P), _I(I), _demandsToRate(demandsToRate)
            {
                // Zero-out previous value for D term
                _lastError = 0;

                // Convert degree parameters to radians for use later
                _bigAngularVel = Filter::deg2rad(BIG_DEGREES_PER_SECOND);

                // Initialize gyro error integral
                resetIntegral();
            }

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                (void)currentTime;

                // P term comes directly from RC command, and D term is zero
                float error = demands.yaw * _demandsToRate - state.angularVel[2];
                float ITermGyro = computeITerm(error, demands.yaw, state.angularVel[2]);

                // Reset integral on large yaw command
                if (fabs(demands.yaw) > BIG_YAW_DEMAND) {
                    ITermGyro = 0;
                    _errorI = 0;
                }

                demands.yaw = computePid(demands.yaw, ITermGyro, 0, state.angularVel[2]);

                // Prevent "yaw jump" during gyroYawPid correction
                demands.yaw = Filter::constrainAbs(demands.yaw, 0.1 + fabs(demands.yaw));

                // We've always gotta do this!
                return true;
            }

            virtual void updateReceiver(demands_t & demands, bool throttleIsDown) override
            {
                // When landed, reset integral component of PID
                if (throttleIsDown) {
                    resetIntegral();
                }
            }

    };  // class YawPid

} // namespace hf
