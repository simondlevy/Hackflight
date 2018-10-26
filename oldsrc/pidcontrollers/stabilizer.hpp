/*
   stabilizer.hpp : PID-based stabilization 

   Copyright (c) 2018 Simon D. Levy

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

    class Stabilizer : public PID_Controller {

        friend class Hackflight;

        private: 

            // Arbitrary constants
            const float GYRO_WINDUP_MAX             = 16.0f;
            const float BIG_GYRO_DEGREES_PER_SECOND = 40.0f; 
            const float BIG_YAW_DEMAND              = 0.1f;
            const float MAX_ARMING_ANGLE_DEGREES    = 25.0f;         

            // PID constants set in constructor
            float _levelP;
            float _gyroCyclicP;
            float _gyroCyclicI;
            float _gyroCyclicD; 
            float _gyroYawP; 
            float _gyroYawI;

            float _lastGyro[2];
            float _gyroDelta1[2]; 
            float _gyroDelta2[2];
            float _errorGyroI[3];

            // For PTerm computation
            float _PTerm[2]; // roll, pitch
            float _demandRoll;
            float _demandPitch;

            // proportion of cyclic demand compared to its maximum
            float _proportionalCyclicDemand;

            float _bigGyroRate;

            float degreesToRadians(float deg)
            {
                return M_PI * deg / 180.;
            }

            float computeITermGyro(float rateP, float rateI, float rcCommand, float gyro[3], uint8_t axis)
            {
                float error = rcCommand*rateP - gyro[axis];

                // Avoid integral windup
                _errorGyroI[axis] = Filter::constrainAbs(_errorGyroI[axis] + error, GYRO_WINDUP_MAX);

                // Reset integral on quick gyro change or large gyroYaw command
                if ((fabs(gyro[axis]) > _bigGyroRate) || ((axis == AXIS_YAW) && (fabs(rcCommand) > BIG_YAW_DEMAND)))
                    _errorGyroI[axis] = 0;

                return (_errorGyroI[axis] * rateI);
            }

            float computePid(float rateP, float PTerm, float ITerm, float DTerm, float gyro[3], uint8_t axis)
            {
                PTerm -= gyro[axis] * rateP; 

                return PTerm + ITerm - DTerm;
            }

            // Computes leveling PID for pitch or roll
            void computeCyclicPTerm(float demand, float eulerAngles[3], uint8_t imuAxis, uint8_t auxState)
            {
                if (auxState == 0) {
                    _PTerm[imuAxis] = demand; 
                }

                else {

                    _PTerm[imuAxis] = (demand - eulerAngles[imuAxis]) * _levelP;  
                    _PTerm[imuAxis] = Filter::complementary(demand, _PTerm[imuAxis], _proportionalCyclicDemand); 
                }
            }

            // Computes leveling PID for pitch or roll
            float computeCyclicPid(float rcCommand, float gyro[3], uint8_t imuAxis)
            {
                // I
                float ITerm = computeITermGyro(_gyroCyclicP, _gyroCyclicI, rcCommand, gyro, imuAxis);
                ITerm *= _proportionalCyclicDemand;

                // D
                float _gyroDelta = gyro[imuAxis] - _lastGyro[imuAxis];
                _lastGyro[imuAxis] = gyro[imuAxis];
                float _gyroDeltaSum = _gyroDelta1[imuAxis] + _gyroDelta2[imuAxis] + _gyroDelta;
                _gyroDelta2[imuAxis] = _gyroDelta1[imuAxis];
                _gyroDelta1[imuAxis] = _gyroDelta;
                float DTerm = _gyroDeltaSum * _gyroCyclicD; 

                return computePid(_gyroCyclicP, _PTerm[imuAxis], ITerm, DTerm, gyro, imuAxis);
            }

            float constrainCyclicDemand(float eulerAngle, float demand)
            {
                return demand * (1 - fabs(eulerAngle)/maxArmingAngle);
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

            Stabilizer(float levelP, float gyroCyclicP, float gyroCyclicI, float gyroCyclicD, float gyroYawP, float gyroYawI) :
                _levelP(levelP), 
                _gyroCyclicP(gyroCyclicP), 
                _gyroCyclicI(gyroCyclicI), 
                _gyroCyclicD(gyroCyclicD), 
                _gyroYawP(gyroYawP), 
                _gyroYawI(gyroYawI) 
            {                // Zero-out previous values for D term
                for (uint8_t axis=0; axis<2; ++axis) {
                    _lastGyro[axis] = 0;
                    _gyroDelta1[axis] = 0;
                    _gyroDelta2[axis] = 0;
                }

                // Convert degree parameters to radians for use later
                _bigGyroRate = degreesToRadians(BIG_GYRO_DEGREES_PER_SECOND);
                maxArmingAngle = degreesToRadians(MAX_ARMING_ANGLE_DEGREES);

                // Initialize gyro error integral
                resetIntegral();
            }

            void updateEulerAngles(float eulerAngles[3], uint8_t auxState)
            {
                computeCyclicPTerm(_demandRoll,  eulerAngles, 0, auxState);
                computeCyclicPTerm(_demandPitch, eulerAngles, 1, auxState);
            }

            void updateReceiver(demands_t & demands, bool throttleIsDown)
            {
                _demandRoll  = demands.roll;
                _demandPitch = demands.pitch;

                // Compute proportion of cyclic demand compared to its maximum
                _proportionalCyclicDemand = Filter::max(fabs(_demandRoll), fabs(_demandPitch)) / 0.5f;
                
                // When landed, reset integral component of PID
                if (throttleIsDown) {
                    resetIntegral();
                }

            }

            bool modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                (void)currentTime;

                // Pitch, roll use leveling based on Euler angles
                demands.roll  = computeCyclicPid(demands.roll,  state.angularVelocities, AXIS_ROLL);
                demands.pitch = computeCyclicPid(demands.pitch, state.angularVelocities, AXIS_PITCH);

                // For gyroYaw, P term comes directly from RC command, and D term is zero
                float ITermGyroYaw = computeITermGyro(_gyroYawP, _gyroYawI, demands.yaw, state.angularVelocities, AXIS_YAW);
                demands.yaw = computePid(_gyroYawP, demands.yaw, ITermGyroYaw, 0, state.angularVelocities, AXIS_YAW);

                // Prevent "gyroYaw jump" during gyroYaw correction
                demands.yaw = Filter::constrainAbs(demands.yaw, 0.1 + fabs(demands.yaw));

                // We've always gotta do this!
                return true;
            }

    };  // class Stabilize

} // namespace
