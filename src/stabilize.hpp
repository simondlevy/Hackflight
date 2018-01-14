/*
   stabilize.hpp : PID-based stablization code

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
#include "filter.hpp"
#include "model.hpp"
#include "debug.hpp"

namespace hf {

    // shared with Hackflight class
    enum {
        AXIS_ROLL = 0,
        AXIS_PITCH,
        AXIS_YAW
    };

    class Stabilize {

        private: // constants

            // Resetting thresholds for PID Integral term
            const float gyroWindupMax           = 16.0f;
            const float bigGyroDegreesPerSecond = 40.0f; 
            const float bigYawDemand            = 0.1f;
            const float maxArmingAngleDegrees   = 25.0f;         

        public:

            void init(Model * _model);

            void update(float demands[4], float eulerAngles[3], float gyroRate[3]);

            void resetIntegral(void);

            float pidRoll;
            float pidPitch;
            float pidYaw;

            float maxArmingAngle;

        private:

            float lastGyro[2];
            float delta1[2]; 
            float delta2[2];
            float errorGyroI[3];

            Board * board;
            Model * model;

            float bigGyroRate;

            float computeITermGyro(float rateP, float rateI, float rcCommand, float gyroRate[3], uint8_t axis);

            float computePid(float rateP, float softwareTrim, float PTerm, float ITerm, float DTerm, float gyroRate[3], uint8_t axis);

            float computeCyclicPid(float rcCommand, float softwareTrim, float prop, float eulerAngles[3],  float gyroRate[3], uint8_t imuAxis);

            float constrainCyclicDemand(float eulerAngle, float demand);

            static float degreesToRadians(float deg);
    }; 


    /********************************************* CPP ********************************************************/

    void Stabilize::init(Model * _model)
    {
        // We'll use PID, IMU config values in update() below
        model = _model;

        // Zero-out previous values for D term
        for (uint8_t axis=0; axis<2; ++axis) {
            lastGyro[axis] = 0;
            delta1[axis] = 0;
            delta2[axis] = 0;
        }

        // Convert degree parameters to radians for use later
        bigGyroRate = degreesToRadians(bigGyroDegreesPerSecond);
        maxArmingAngle = degreesToRadians(maxArmingAngleDegrees);

        // Initialize PIDs
        pidRoll = 0;
        pidPitch = 0;
        pidYaw = 0;

        // Initialize gyro error integral
        resetIntegral();
    }

    float Stabilize::degreesToRadians(float deg)
    {
        return M_PI * deg / 180.;
    }

    float Stabilize::computeITermGyro(float rateP, float rateI, float rcCommand, float gyroRate[3], uint8_t axis)
    {
        float error = rcCommand*rateP - gyroRate[axis];

        // Avoid integral windup
        errorGyroI[axis] = Filter::constrainAbs(errorGyroI[axis] + error, gyroWindupMax);

        // Reset integral on quick gyro change or large gyroYaw command
        if ((fabs(gyroRate[axis]) > bigGyroRate) || ((axis == AXIS_YAW) && (fabs(rcCommand) > bigYawDemand)))
            errorGyroI[axis] = 0;

        return (errorGyroI[axis] * rateI);
    }

    float Stabilize::computePid(
            float rateP, 
            float softwareTrim,
            float PTerm, 
            float ITerm, 
            float DTerm, 
            float gyroRate[3], 
            uint8_t axis)
    {
        PTerm -= gyroRate[axis] * rateP; 

        return PTerm + ITerm - DTerm + softwareTrim;
    }

    // Computes leveling PID for pitch or roll
    float Stabilize::computeCyclicPid(
            float rcCommand, 
            float softwareTrim,
            float prop, 
            float eulerAngles[3], 
            float gyroRate[3], 
            uint8_t imuAxis)
    {
        float ITermGyro = computeITermGyro(model->gyroCyclicP, model->gyroCyclicI, rcCommand, gyroRate, imuAxis);

        float PTermAccel = (rcCommand - eulerAngles[imuAxis]) * model->levelP;  

        float PTerm = Filter::complementary(rcCommand, PTermAccel, prop); 

        float ITerm = ITermGyro * prop;

        float delta = gyroRate[imuAxis] - lastGyro[imuAxis];
        lastGyro[imuAxis] = gyroRate[imuAxis];
        float deltaSum = delta1[imuAxis] + delta2[imuAxis] + delta;
        delta2[imuAxis] = delta1[imuAxis];
        delta1[imuAxis] = delta;
        float DTerm = deltaSum * model->gyroCyclicD; 

        return computePid(model->gyroCyclicP, softwareTrim, PTerm, ITerm, DTerm, gyroRate, imuAxis);
    }

    float Stabilize::constrainCyclicDemand(float eulerAngle, float demand)
    {
        return fabs(eulerAngle) > maxArmingAngle ? 0 : demand;
    }

    void Stabilize::update(float demands[4], float eulerAngles[3], float gyroRate[3])
    {
        // Compute proportion of cyclic demand compared to its maximum
        float prop = Filter::max(fabs(demands[Receiver::DEMAND_ROLL]), fabs(demands[Receiver::DEMAND_PITCH])) / 0.5f;

        // In level mode, reject pitch, roll demands that increase angle beyond specified maximum
        if (model->levelP > 0) {
            demands[Receiver::DEMAND_ROLL]  = constrainCyclicDemand(eulerAngles[0], demands[Receiver::DEMAND_ROLL]);
            demands[Receiver::DEMAND_PITCH] = constrainCyclicDemand(eulerAngles[1], demands[Receiver::DEMAND_PITCH]);
        }

        // Pitch, roll use leveling based on Euler angles

        pidRoll = computeCyclicPid(demands[Receiver::DEMAND_ROLL],  model->softwareTrimRoll,  prop, 
                eulerAngles, gyroRate, AXIS_ROLL);

        pidPitch = computeCyclicPid(demands[Receiver::DEMAND_PITCH], model->softwareTrimPitch, prop, 
                eulerAngles, gyroRate, AXIS_PITCH);

        Debug::printf("%f %f", pidRoll, eulerAngles[0]);

        // For gyroYaw, P term comes directly from RC command, and D term is zero
        float ITermGyroYaw = computeITermGyro(model->gyroYawP, model->gyroYawI, demands[Receiver::DEMAND_YAW], gyroRate, AXIS_YAW);
        pidYaw = computePid(model->gyroYawP, model->softwareTrimYaw, demands[Receiver::DEMAND_YAW], ITermGyroYaw, 0, gyroRate, AXIS_YAW);

        // Prevent "gyroYaw jump" during gyroYaw correction
        pidYaw = Filter::constrainAbs(pidYaw, 0.1 + fabs(demands[Receiver::DEMAND_YAW]));
    }

    void Stabilize::resetIntegral(void)
    {
        errorGyroI[AXIS_ROLL] = 0;
        errorGyroI[AXIS_PITCH] = 0;
        errorGyroI[AXIS_YAW] = 0;
    }

} // namespace
