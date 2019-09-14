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
            const float GYRO_WINDUP_MAX             = 6.0f;
            const float BIG_GYRO_DEGREES_PER_SECOND = 40.0f; 
            const float BIG_YAW_DEMAND              = 0.1f;

            // Converted to radians from degrees in init() method for efficiency
            float _bigGyroRate = 0;

            // PID constants set in constructor
            float _P = 0; 
            float _I = 0;

            float computeITermGyro(float error, float rateI, float rcCommand, float gyro[3])
            {
                // Avoid integral windup
                _errorGyroI = Filter::constrainAbs(_errorGyroI + error, GYRO_WINDUP_MAX);

                // Reset integral on quick gyro change or large gyroYawPid command
                if ((fabs(gyro[2]) > _bigGyroRate) || (fabs(rcCommand) > BIG_YAW_DEMAND)) {
                    _errorGyroI = 0;
                }

                return _errorGyroI * rateI;
            }

            void init(void)
            {
              // Zero-out previous value for D term
              _lastError = 0;

              // Convert degree parameters to radians for use later
              _bigGyroRate   = Filter::deg2rad(BIG_GYRO_DEGREES_PER_SECOND);

              // Initialize gyro error integral
              resetIntegral();
            }

            float _lastError = 0;
            float _errorGyroI = 0;

            float computePid(float rateP, float PTerm, float ITerm, float DTerm, float angularVel)
            {
                PTerm = (PTerm * _demandsToRate - angularVel) * rateP;

                return PTerm + ITerm + DTerm;
            }

        protected:

            float _demandsToRate;

            // proportion of cyclic demand compared to its maximum
            float _proportionalCyclicDemand;

            void resetIntegral(void)
            {
                _errorGyroI = 0;
            }

        public:

            YawPid(float P, float I, float demandsToRate = 1.0f) 
                : _P(P), _I(I), _demandsToRate(demandsToRate)
            {
                init();
            }

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                (void)currentTime;

                // P term comes directly from RC command, and D term is zero
                float error = demands.yaw * _demandsToRate - state.angularVel[AXIS_YAW];
                float ITermGyro = computeITermGyro(error, _I, demands.yaw, state.angularVel);
                demands.yaw = computePid(_P, demands.yaw, ITermGyro, 0, state.angularVel[2]);

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
