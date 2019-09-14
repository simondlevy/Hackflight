/*
   acro.hpp : acro-mode PID controller 

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

    class AcroPid : public PidController {

        friend class Hackflight;

        private: 

            // Arbitrary constants
            static constexpr float WINDUP_MAX             = 6.0f;
            static constexpr float BIG_DEGREES_PER_SECOND = 40.0f; 

            float _lastError[2];
            float _deltaError1[2]; 
            float _deltaError2[2];
            float _errorI[2];

            // Arrays of PID constants for pitch and roll
            float _PConstants[2];
            float _IConstants[2];
            float _DConstants[2];

            // Converted to radians from degrees in init() method for efficiency
            float _bigRate = 0;

            float computeITermGyro(float error, float rateI, float rcCommand, float rate[3], uint8_t axis)
            {
                // Avoid integral windup
                _errorI[axis] = Filter::constrainAbs(_errorI[axis] + error, WINDUP_MAX);

                // Reset integral on quick gyro change
                if (fabs(rate[axis]) > _bigRate)
                    _errorI[axis] = 0;

                return (_errorI[axis] * rateI);
            }

            void init(void)
            {
              // Zero-out previous values for D term
              for (uint8_t axis=0; axis<2; ++axis) {
                  _lastError[axis] = 0;
                  _deltaError1[axis] = 0;
                  _deltaError2[axis] = 0;
              }

              // Convert degree parameters to radians for use later
              _bigRate = Filter::deg2rad(BIG_DEGREES_PER_SECOND);

              // Initialize gyro error integral
              resetIntegral();
            }

            // Computes PID for pitch or roll
            float computeCyclicPid(float rcCommand, float rate[3], uint8_t imuAxis)
            {
                float error = rcCommand * _demandsScale - rate[imuAxis];

                // I
                float ITerm = computeITermGyro(error, _IConstants[imuAxis], rcCommand, gyro, imuAxis);
                ITerm *= _proportionalCyclicDemand;

                // D
                float deltaError = error - _lastError[imuAxis];
                _lastError[imuAxis] = error;
                float deltaErrorSum = _deltaError1[imuAxis] + _deltaError2[imuAxis] + deltaError;
                _deltaError2[imuAxis] = _deltaError1[imuAxis];
                _deltaError1[imuAxis] = deltaError;
                float DTerm = deltaErrorSum * _DConstants[imuAxis]; 

                return computePid(_PConstants[imuAxis], _PTerm[imuAxis], ITerm, DTerm, gyro, imuAxis);
            }

            
            float computePid(float rateP, float PTerm, float ITerm, float DTerm, float rate[3], uint8_t axis)
            {
                PTerm = (PTerm * _demandsScale - rate[axis]) * rateP;

                return PTerm + ITerm + DTerm;
            }

            float maxval(float a, float b)
            {
                return a > b ? a : b;
            }

        protected:

            // For PTerm computation
            float _PTerm[2]; // roll, pitch

            float _demandsScale;

            // proportion of cyclic demand compared to its maximum
            float _proportionalCyclicDemand;

            void resetIntegral(void)
            {
                _errorI[AXIS_ROLL] = 0;
                _errorI[AXIS_PITCH] = 0;
            }

        public:

            AcroPid(float P, float I, float D,float demandsScale=1.0f) :

                _demandsScale(demandsScale)
            {
                init();
            
                _PConstants[0] = P;
                _PConstants[1] = P;
                _IConstants[0] = I;
                _IConstants[1] = I;
                _DConstants[0] = D;
                _DConstants[1] = D;
            }

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                (void)currentTime;

                _PTerm[0] = demands.roll;
                _PTerm[1] = demands.pitch;

                // Pitch, roll use Euler angles
                demands.roll  = computeCyclicPid(demands.roll,  state.angularVel, AXIS_ROLL);
                demands.pitch = computeCyclicPid(demands.pitch, state.angularVel, AXIS_PITCH);

                // We've always gotta do this!
                return true;
            }

            virtual void updateReceiver(demands_t & demands, bool throttleIsDown) override
            {
                // Compute proportion of cyclic demand compared to its maximum
                _proportionalCyclicDemand = maxval(fabs(demands.roll), fabs(demands.pitch)) / 0.5f;
                
                // When landed, reset integral component of PID
                if (throttleIsDown) {
                    resetIntegral();
                }
            }

    };  // class AcroPid

} // namespace hf
