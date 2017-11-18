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

#include "receiver.hpp"
#include "config.hpp"
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

public:

    void init(const StabilizeConfig& _config, const ImuConfig& _imuConfig, Model * _model);

    void update(float rcCommandRoll, float rcCommandPitch, float rcCommandYaw, 
            float eulerAnglesRadians[3], float gyroRadiansPerSecond[3]);

    void resetIntegral(void);

    float pidRoll;
    float pidPitch;
    float pidYaw;

private:

    float lastGyro[2];
    float delta1[2]; 
    float delta2[2];
    float errorGyroI[3];

    Board * board;
    Model * model;

    ImuConfig imuConfig;
    StabilizeConfig config;

    float bigGyroRadiansPerSecond;

    float computeITermGyro(float rateP, float rateI, float rcCommand, float gyroRadiansPerSecond[3], uint8_t axis);
    float computePid(float rateP, float softwareTrim, float PTerm, float ITerm, float DTerm, 
            float gyroRadiansPerSecond[3], uint8_t axis);
    float computePitchRollPid(float rcCommand, float softwareTrim, float prop, 
            float eulerAnglesRadians[3],  float gyroRadiansPerSecond[3], uint8_t imuAxis);
}; 


/********************************************* CPP ********************************************************/

void Stabilize::init(const StabilizeConfig& _config, const ImuConfig& _imuConfig, Model * _model)
{
    // We'll use PID, IMU config values in update() below
    memcpy(&config, &_config, sizeof(StabilizeConfig));
    memcpy(&imuConfig, &_imuConfig, sizeof(ImuConfig));
    model = _model;

    // Zero-out previous values for D term
    for (uint8_t axis=0; axis<2; ++axis) {
        lastGyro[axis] = 0;
        delta1[axis] = 0;
        delta2[axis] = 0;
    }

    // Convert big gyro turn speed to radians for use later
    bigGyroRadiansPerSecond = M_PI * config.bigGyro / 180.;

    pidRoll = 0;
    pidPitch = 0;
    pidYaw = 0;

    resetIntegral();
}

float Stabilize::computeITermGyro(float rateP, float rateI, float rcCommand, float gyroRadiansPerSecond[3], uint8_t axis)
{
    float error = rcCommand*rateP - gyroRadiansPerSecond[axis];

    // Avoid integral windup
    errorGyroI[axis] = Filter::constrainAbs(errorGyroI[axis] + error, config.gyroWindupMax);

    // Reset integral on quick gyro change or large yaw command
    if ((std::abs(gyroRadiansPerSecond[axis]) > bigGyroRadiansPerSecond) || 
            ((axis == AXIS_YAW) && (std::abs(rcCommand) > config.bigYawDemand)))
        errorGyroI[axis] = 0;

    return (errorGyroI[axis] * rateI);
}

float Stabilize::computePid(
        float rateP, 
        float softwareTrim,
        float PTerm, 
        float ITerm, 
        float DTerm, 
        float gyroRadiansPerSecond[3], 
        uint8_t axis)
{
    PTerm -= gyroRadiansPerSecond[axis] * rateP; 

    return PTerm + ITerm - DTerm + softwareTrim;
}

// Computes leveling PID for pitch or roll
float Stabilize::computePitchRollPid(
        float rcCommand, 
        float softwareTrim,
        float prop, 
        float eulerAnglesRadians[3], 
        float gyroRadiansPerSecond[3], 
        uint8_t imuAxis)
{
    float ITermGyro = computeITermGyro(model->ratePitchRollP, model->ratePitchRollI, rcCommand, gyroRadiansPerSecond, imuAxis);

    float PTermAccel = (rcCommand - eulerAnglesRadians[imuAxis]) * model->levelP;  

    float PTerm = Filter::complementary(rcCommand, PTermAccel, prop); 

    float ITerm = ITermGyro * prop;

    float delta = gyroRadiansPerSecond[imuAxis] - lastGyro[imuAxis];
    lastGyro[imuAxis] = gyroRadiansPerSecond[imuAxis];
    float deltaSum = delta1[imuAxis] + delta2[imuAxis] + delta;
    delta2[imuAxis] = delta1[imuAxis];
    delta1[imuAxis] = delta;
    float DTerm = deltaSum * model->ratePitchRollD; 

    return computePid(model->ratePitchRollP, softwareTrim, PTerm, ITerm, DTerm, gyroRadiansPerSecond, imuAxis);
}

void Stabilize::update(
        float rcCommandRoll, 
        float rcCommandPitch, 
        float rcCommandYaw, 
        float eulerAnglesRadians[3], 
        float gyroRadiansPerSecond[3])
{
    // Compute proportion of cyclic demand compared to its maximum
    float prop = (std::max)(std::abs(rcCommandRoll), std::abs(rcCommandPitch)) / 0.5f;

    // Pitch, roll use leveling based on Euler angles
    pidRoll  = computePitchRollPid(rcCommandRoll,  model->softwareTrimRoll,  prop, 
            eulerAnglesRadians, gyroRadiansPerSecond, AXIS_ROLL);
    pidPitch = computePitchRollPid(rcCommandPitch, model->softwareTrimPitch, prop, 
            eulerAnglesRadians, gyroRadiansPerSecond, AXIS_PITCH);

    // For yaw, P term comes directly from RC command, and D term is zero
    float ITermGyroYaw = computeITermGyro(model->yawP, model->yawI, rcCommandYaw, gyroRadiansPerSecond, AXIS_YAW);
    pidYaw = computePid(model->yawP, model->softwareTrimYaw, rcCommandYaw, ITermGyroYaw, 0, gyroRadiansPerSecond, AXIS_YAW);

    // Prevent "yaw jump" during yaw correction
    pidYaw = Filter::constrainAbs(pidYaw, 0.1 + std::abs(rcCommandYaw));
}

void Stabilize::resetIntegral(void)
{
    errorGyroI[AXIS_ROLL] = 0;
    errorGyroI[AXIS_PITCH] = 0;
    errorGyroI[AXIS_YAW] = 0;
}

} // namespace
