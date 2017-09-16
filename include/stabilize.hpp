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
#include "common.hpp"
#include "filter.hpp"

namespace hf {

class Stabilize {
public:
    int16_t axisPID[3];

    void init(const StabilizeConfig& _config, const ImuConfig& _imuConfig);

    void update(int16_t rcCommand[4], int16_t gyroADC[3], float eulerAnglesDegrees[3]);

    void resetIntegral(void);

private:

    int16_t lastGyro[2];
    int32_t delta1[2]; 
    int32_t delta2[2];
    int32_t errorGyroI[3];
    int32_t errorAngleI[2];

    Board * board;

    ImuConfig imuConfig;
    StabilizeConfig config;

    int32_t computeITermGyro(float rateP, float rateI, int16_t rcCommand, int16_t gyroADC[3], uint8_t axis);
    int16_t computePid(float rateP, int32_t PTerm, int32_t ITerm, int32_t DTerm, int16_t gyroADC[3], uint8_t axis);
    int16_t computeLevelPid(int16_t rcCommand[4], uint8_t rcAxis, int16_t gyroADC[3], float eulerAnglesDegrees[3], uint8_t imuAxis);
}; 


/********************************************* CPP ********************************************************/

void Stabilize::init(const StabilizeConfig& _config, const ImuConfig& _imuConfig)
{
    // We'll use PID, IMU config values in update() below
    memcpy(&config, &_config, sizeof(StabilizeConfig));
    memcpy(&imuConfig, &_imuConfig, sizeof(ImuConfig));

    // Zero-out previous values for D term
    for (uint8_t axis=0; axis<2; ++axis) {
        lastGyro[axis] = 0;
        delta1[axis] = 0;
        delta2[axis] = 0;
    }

    resetIntegral();
}

int32_t Stabilize::computeITermGyro(float rateP, float rateI, int16_t rcCommand, int16_t gyroADC[3], uint8_t axis)
{
    int32_t error = ((int32_t)rcCommand * rateP) - gyroADC[axis];

    // Avoid integral windup
    errorGyroI[axis] = Filter::constrainAbs(errorGyroI[axis] + error, config.gyroWindupMax);

    // Reset integral on quick gyro change or large yaw command
    if ((std::abs(gyroADC[axis]) > config.bigGyro) || ((axis == AXIS_YAW) && (std::abs(rcCommand) > config.bigYawDemand)))
        errorGyroI[axis] = 0;

    return (int32_t)(errorGyroI[axis] * rateI);
}

int16_t Stabilize::computePid(float rateP, int32_t PTerm, int32_t ITerm, int32_t DTerm, int16_t gyroADC[3], uint8_t axis)
{
    PTerm -= (int32_t)gyroADC[axis] * rateP;
    return PTerm + ITerm - DTerm + config.softwareTrim[axis];
}

// Computes PID for pitch or roll
int16_t Stabilize::computeLevelPid(int16_t rcCommand[4], uint8_t rcAxis, int16_t gyroADC[3], float eulerAnglesDegrees[3], uint8_t imuAxis)
{
    int32_t ITermGyro = computeITermGyro(config.ratePitchrollP, config.ratePitchrollI, rcCommand[rcAxis], gyroADC, imuAxis);

    // RC command is in [-500,+500].  We compute error by scaling it up to [-1000,+1000], then treating this value as tenths
    // of a degree and subtracting off corresponding pitch or roll angle obtained from IMU.
    int32_t errorAngle = Filter::constrainAbs(2*rcCommand[rcAxis], 10*imuConfig.maxAngleInclination) - 10*eulerAnglesDegrees[imuAxis];

    int32_t PTermAccel = errorAngle * config.levelP; 

    // Avoid integral windup
    errorAngleI[imuAxis] = Filter::constrainAbs(errorAngleI[imuAxis] + errorAngle, config.angleWindupMax);

    // Compute proportion of cyclic demand compared to its maximum
    float prop = (std::max)(std::abs(rcCommand[DEMAND_PITCH]), std::abs(rcCommand[DEMAND_ROLL])) / 500.f;

    int32_t PTerm = Filter::complementary(rcCommand[rcAxis], PTermAccel, prop);

    int32_t ITerm = ITermGyro * prop;

    int32_t delta = gyroADC[imuAxis] - lastGyro[imuAxis];
    lastGyro[imuAxis] = gyroADC[imuAxis];
    int32_t deltaSum = delta1[imuAxis] + delta2[imuAxis] + delta;
    delta2[imuAxis] = delta1[imuAxis];
    delta1[imuAxis] = delta;
    int32_t DTerm = deltaSum * config.ratePitchrollD;

    return computePid(config.ratePitchrollP, PTerm, ITerm, DTerm, gyroADC, imuAxis);
}

void Stabilize::update(int16_t rcCommand[4], int16_t gyroADC[3], float eulerAnglesDegrees[3])
{
    // Pitch, roll use leveling based on Euler angles
    axisPID[AXIS_ROLL]  = computeLevelPid(rcCommand, DEMAND_ROLL,  gyroADC, eulerAnglesDegrees, AXIS_ROLL);
    axisPID[AXIS_PITCH] = computeLevelPid(rcCommand, DEMAND_PITCH, gyroADC, eulerAnglesDegrees, AXIS_PITCH);

    // For yaw, P term comes directly from RC command, and D term is zero
    int32_t ITermGyroYaw = computeITermGyro(config.yawP, config.yawI, rcCommand[DEMAND_YAW], gyroADC, AXIS_YAW);
    axisPID[AXIS_YAW] = computePid(config.yawP, rcCommand[DEMAND_YAW], ITermGyroYaw, 0, gyroADC, AXIS_YAW);

    // Prevent "yaw jump" during yaw correction
    axisPID[AXIS_YAW] = Filter::constrainAbs(axisPID[AXIS_YAW], 100 + std::abs(rcCommand[DEMAND_YAW]));
}

void Stabilize::resetIntegral(void)
{
    errorGyroI[AXIS_ROLL] = 0;
    errorGyroI[AXIS_PITCH] = 0;
    errorGyroI[AXIS_YAW] = 0;
    errorAngleI[AXIS_ROLL] = 0;
    errorAngleI[AXIS_PITCH] = 0;
}

} // namespace
