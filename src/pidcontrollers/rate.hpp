/*
   rate.hpp : rate PID controller

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

    // shared with Hackflight class
    enum {
        AXIS_ROLL = 0,
        AXIS_PITCH,
        AXIS_YAW
    };

    class Rate : public PID_Controller {

        friend class Hackflight;

        private: 

            // Arbitrary constants
            const float GYRO_WINDUP_MAX             = 6.0f;
            const float BIG_GYRO_DEGREES_PER_SECOND = 40.0f; 
            const float BIG_YAW_DEMAND              = 0.1f;

            // Converted to radians from degrees in init() method for efficiency
            float _bigGyroRate = 0;

            // PID constants set in constructor
            float _gyroYawP = 0; 
            float _gyroYawI = 0;

            float computeITermGyro(float error, float rateI, float rcCommand, float gyro[3], uint8_t axis)
            {
                // Avoid integral windup
                _errorGyroI[axis] = Filter::constrainAbs(_errorGyroI[axis] + error, GYRO_WINDUP_MAX);

                // Reset integral on quick gyro change or large gyroYaw command
                if ((fabs(gyro[axis]) > _bigGyroRate) || ((axis == AXIS_YAW) && (fabs(rcCommand) > BIG_YAW_DEMAND)))
                    _errorGyroI[axis] = 0;

                return (_errorGyroI[axis] * rateI);
            }

            void init(void)
            {
              // Zero-out previous values for D term
              for (uint8_t axis=0; axis<2; ++axis) {
                  _lastError[axis] = 0;
                  _gyroDeltaError1[axis] = 0;
                  _gyroDeltaError2[axis] = 0;
              }

              // Convert degree parameters to radians for use later
              _bigGyroRate   = Filter::deg2rad(BIG_GYRO_DEGREES_PER_SECOND);

              // Initialize gyro error integral
              resetIntegral();
            }

            float _lastError[2];
            float _gyroDeltaError1[2]; 
            float _gyroDeltaError2[2];
            float _errorGyroI[3];

            // Arrays of PID constants for pitch and roll
            float _PConstants[2];
            float _IConstants[2];
            float _DConstants[2];

            // Computes PID for pitch or roll
            float computeCyclicPid(float rcCommand, float gyro[3], uint8_t imuAxis)
            {
                float error = rcCommand * _demandsToRate - gyro[imuAxis];

                // I
                float ITerm = computeITermGyro(error, _IConstants[imuAxis], rcCommand, gyro, imuAxis);
                ITerm *= _proportionalCyclicDemand;

                // D
                float gyroDeltaError = error - _lastError[imuAxis];
                _lastError[imuAxis] = error;
                float gyroDeltaErrorSum = _gyroDeltaError1[imuAxis] + _gyroDeltaError2[imuAxis] + gyroDeltaError;
                _gyroDeltaError2[imuAxis] = _gyroDeltaError1[imuAxis];
                _gyroDeltaError1[imuAxis] = gyroDeltaError;
                float DTerm = gyroDeltaErrorSum * _DConstants[imuAxis]; 

                return computePid(_PConstants[imuAxis], _PTerm[imuAxis], ITerm, DTerm, gyro, imuAxis);
            }

            
            float computePid(float rateP, float PTerm, float ITerm, float DTerm, float gyro[3], uint8_t axis)
            {
                PTerm = (PTerm * _demandsToRate - gyro[axis]) * rateP;

                return PTerm + ITerm + DTerm;
            }

            float maxval(float a, float b)
            {
                return a > b ? a : b;
            }

        protected:

            // For PTerm computation
            float _PTerm[2]; // roll, pitch

            float _demandsToRate;

            // proportion of cyclic demand compared to its maximum
            float _proportionalCyclicDemand;

            void resetIntegral(void)
            {
                _errorGyroI[AXIS_ROLL] = 0;
                _errorGyroI[AXIS_PITCH] = 0;
                _errorGyroI[AXIS_YAW] = 0;
            }

        public:

            Rate(float gyroRollPitchP, float gyroRollPitchI, float gyroRollPitchD,
                       float gyroYawP, float gyroYawI, float demandsToRate = 1.0f) :

                _gyroYawP(gyroYawP), 
                _gyroYawI(gyroYawI), 
                _demandsToRate(demandsToRate)
            {
                init();
            
                _PConstants[0] = gyroRollPitchP;
                _PConstants[1] = gyroRollPitchP;
                _IConstants[0] = gyroRollPitchI;
                _IConstants[1] = gyroRollPitchI;
                _DConstants[0] = gyroRollPitchD;
                _DConstants[1] = gyroRollPitchD;
            }

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                (void)currentTime;

                _PTerm[0] = demands.roll;
                _PTerm[1] = demands.pitch;

                // Pitch, roll use Euler angles
                demands.roll  = computeCyclicPid(demands.roll,  state.angularVel, AXIS_ROLL);
                demands.pitch = computeCyclicPid(demands.pitch, state.angularVel, AXIS_PITCH);

                // For gyroYaw, P term comes directly from RC command, and D term is zero
                float yawError = demands.yaw * _demandsToRate - state.angularVel[AXIS_YAW];
                float ITermGyroYaw = computeITermGyro(yawError, _gyroYawI, demands.yaw, state.angularVel, AXIS_YAW);
                demands.yaw = computePid(_gyroYawP, demands.yaw, ITermGyroYaw, 0, state.angularVel, AXIS_YAW);

                // Prevent "gyroYaw jump" during gyroYaw correction
                demands.yaw = Filter::constrainAbs(demands.yaw, 0.1 + fabs(demands.yaw));

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

    };  // class Rate

} // namespace hf
