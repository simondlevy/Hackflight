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

#include <cstdint>
#include <cstring>
#include <algorithm>
#include <limits>
#include <cmath>

#include "receiver.hpp"
#include "filters.hpp"
#include "debug.hpp"
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
            const float MAX_ARMING_ANGLE_DEGREES    = 25.0f;

            // PID constants set in constructor
            float _demandsToRate;
            float _gyroYawP; 
            float _gyroYawI;
            // Arrays of PID constants for pitch and roll
            float _PConstants[2];
            float _IConstants[2];
            float _DConstants[2];


            float _lastError[2];
            float _gyroDeltaError1[2]; 
            float _gyroDeltaError2[2];
            float _errorGyroI[3];

            // For PTerm computation
            float _PTerm[2]; // roll, pitch

            // proportion of cyclic demand compared to its maximum
            float _proportionalCyclicDemand;

            float _bigGyroRate;
            
            void init(void)
            {
              // Zero-out previous values for D term
              for (uint8_t axis=0; axis<2; ++axis) {
                  _lastError[axis] = 0;
                  _gyroDeltaError1[axis] = 0;
                  _gyroDeltaError2[axis] = 0;
              }

              // Convert degree parameters to radians for use later
              _bigGyroRate = degreesToRadians(BIG_GYRO_DEGREES_PER_SECOND);
              maxArmingAngle = degreesToRadians(MAX_ARMING_ANGLE_DEGREES);

              // Initialize gyro error integral
              resetIntegral();

            }

            float degreesToRadians(float deg)
            {
                return M_PI * deg / 180.;
            }

            float computeITermGyro(float error, float rateI, float rcCommand, float gyro[3], uint8_t axis)
            {
                // Avoid integral windup
                _errorGyroI[axis] = Filter::constrainAbs(_errorGyroI[axis] + error, GYRO_WINDUP_MAX);

                // Reset integral on quick gyro change or large gyroYaw command
                if ((fabs(gyro[axis]) > _bigGyroRate) || ((axis == AXIS_YAW) && (fabs(rcCommand) > BIG_YAW_DEMAND)))
                    _errorGyroI[axis] = 0;

                return (_errorGyroI[axis] * rateI);
            }

            float computePid(float rateP, float PTerm, float ITerm, float DTerm, float gyro[3], uint8_t axis)
            {
                PTerm = (PTerm * _demandsToRate -gyro[axis]) * rateP;

                return PTerm + ITerm + DTerm;
            }

            // Computes leveling PID for pitch or roll
            float computeCyclicPid(float rcCommand, float gyro[3], uint8_t imuAxis)
            {
                float error = rcCommand * _demandsToRate - gyro[imuAxis];
                // I
                float ITerm = computeITermGyro(error, _IConstants[imuAxis], rcCommand, gyro, imuAxis);
                ITerm *= _proportionalCyclicDemand;

                // D
                float _gyroDeltaError = error - _lastError[imuAxis];
                _lastError[imuAxis] = error;
                float _gyroDeltaErrorSum = _gyroDeltaError1[imuAxis] + _gyroDeltaError2[imuAxis] + _gyroDeltaError;
                _gyroDeltaError2[imuAxis] = _gyroDeltaError1[imuAxis];
                _gyroDeltaError1[imuAxis] = _gyroDeltaError;
                float DTerm = _gyroDeltaErrorSum * _DConstants[imuAxis]; 

                return computePid(_PConstants[imuAxis], _PTerm[imuAxis], ITerm, DTerm, gyro, imuAxis);
            }

            void resetIntegral(void)
            {
                _errorGyroI[AXIS_ROLL] = 0;
                _errorGyroI[AXIS_PITCH] = 0;
                _errorGyroI[AXIS_YAW] = 0;
            }

        protected:

            float maxArmingAngle;

        public:

            Rate(float gyroRollP, float gyroRollI, float gyroRollD,
                       float gyroPitchP, float gyroPitchI, float gyroPitchD,
                       float gyroYawP, float gyroYawI, float demandsToRate = 1.0f) :
                _demandsToRate(demandsToRate),
                _gyroYawP(gyroYawP), 
                _gyroYawI(gyroYawI) 
            {
                init();
                // Constants arrays
                _PConstants[0] = gyroRollP;
                _PConstants[1] = gyroPitchP;
                _IConstants[0] = gyroRollI;
                _IConstants[1] = gyroPitchI;
                _DConstants[0] = gyroRollD;
                _DConstants[1] = gyroPitchD;
            }
            
            Rate(float gyroRollPitchP, float gyroRollPitchI, float gyroRollPitchD,
                       float gyroYawP, float gyroYawI, float demandsToRate = 1.0f) :
                _demandsToRate(demandsToRate),
                _gyroYawP(gyroYawP), 
                _gyroYawI(gyroYawI) 
            {
                init();
                // Constants arrays
                _PConstants[0] = gyroRollPitchP;
                _PConstants[1] = gyroRollPitchP;
                _IConstants[0] = gyroRollPitchI;
                _IConstants[1] = gyroRollPitchI;
                _DConstants[0] = gyroRollPitchD;
                _DConstants[1] = gyroRollPitchD;
            }

            void updateReceiver(demands_t & demands, bool throttleIsDown)
            {
                // Compute proportion of cyclic demand compared to its maximum
                _proportionalCyclicDemand = Filter::max(fabs(demands.roll), fabs(demands.pitch)) / 0.5f;
                
                // When landed, reset integral component of PID
                if (throttleIsDown) {
                    resetIntegral();
                }
            }

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                (void)currentTime;

                _PTerm[0] = demands.roll;
                _PTerm[1] = demands.pitch;

                // Pitch, roll use leveling based on Euler angles
                demands.roll  = computeCyclicPid(demands.roll,  state.angularVelocities, AXIS_ROLL);
                demands.pitch = computeCyclicPid(demands.pitch, state.angularVelocities, AXIS_PITCH);

                // For gyroYaw, P term comes directly from RC command, and D term is zero
                float yawError = demands.yaw * _demandsToRate - state.angularVelocities[AXIS_YAW];
                float ITermGyroYaw = computeITermGyro(yawError, _gyroYawI, demands.yaw, state.angularVelocities, AXIS_YAW);
                demands.yaw = computePid(_gyroYawP, demands.yaw, ITermGyroYaw, 0, state.angularVelocities, AXIS_YAW);

                // Prevent "gyroYaw jump" during gyroYaw correction
                demands.yaw = Filter::constrainAbs(demands.yaw, 0.1 + fabs(demands.yaw));

                // We've always gotta do this!
                return true;
            }

    };  // class Stabilize

} // namespace
