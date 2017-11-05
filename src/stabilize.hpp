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

/*
#include <stdarg.h>
static void  dprintf(const char * fmt, ...)
{ 
    va_list ap;
    va_start(ap, fmt);
    char buf[200];
    vsprintf(buf, fmt, ap);
    for (char* p=buf; *p; p++) {
        Serial.print(*p);
    }
    va_end(ap);
}
*/

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
            float eulerAnglesDegrees[3], float gyroDegreesPerSecond[3]);

    void resetIntegral(void);

    float pidRoll;
    float pidPitch;
    float pidYaw;

private:

    float lastGyro[2];
    float delta1[2]; 
    float delta2[2];
    float errorGyroI[3];
    float errorAngleI[2];

    Board * board;
    Model * model;

    ImuConfig imuConfig;
    StabilizeConfig config;

    float computeITermGyro(float rateP, float rateI, float rcCommand, float gyroDegreesPerSecond[3], uint8_t axis);
    float computePid(float rateP, float softwareTrim, float PTerm, float ITerm, float DTerm, 
            float gyroDegreesPerSecond[3], uint8_t axis);
    float computePitchRollPid(float rcCommand, float softwareTrim, float prop, 
            float eulerAnglesDegrees[3],  float gyroDegreesPerSecond[3], uint8_t imuAxis);

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

    pidRoll = 0;
    pidPitch = 0;
    pidYaw = 0;

    resetIntegral();
}

float Stabilize::computeITermGyro(float rateP, float rateI, float rcCommand, float gyroDegreesPerSecond[3], uint8_t axis)
{
    float error = (float)(rcCommand * rateP) - 16*gyroDegreesPerSecond[axis]; // XXX

    // Avoid integral windup
    errorGyroI[axis] = Filter::constrainAbs(errorGyroI[axis] + error, config.gyroWindupMax);

    // Reset integral on quick gyro change or large yaw command
    if ((std::abs(gyroDegreesPerSecond[axis]) > config.bigGyro) || 
            ((axis == AXIS_YAW) && (std::abs(rcCommand) > config.bigYawDemand)))
        errorGyroI[axis] = 0;

    return (float)(errorGyroI[axis] * rateI);
}

float Stabilize::computePid(
        float rateP, 
        float softwareTrim,
        float PTerm, 
        float ITerm, 
        float DTerm, 
        float gyroDegreesPerSecond[3], 
        uint8_t axis)
{
    PTerm -= (16*gyroDegreesPerSecond[axis] * rateP); // XXX 
    return (PTerm + ITerm - DTerm + 1000*softwareTrim) / 1000.; // XXX
}

// Computes leveling PID for pitch or roll
float Stabilize::computePitchRollPid(
        float rcCommandF, 
        float softwareTrim,
        float prop, 
        float eulerAnglesDegrees[3], 
        float gyroDegreesPerSecond[3], 
        uint8_t imuAxis)
{
    float rcCommand = 1000 * rcCommandF;// XXX

    float ITermGyro = computeITermGyro(model->ratePitchRollP, model->ratePitchRollI, rcCommand, 
            gyroDegreesPerSecond, imuAxis);

    // RC command is in [-500,+500].  We compute error by scaling it up to [-1000,+1000], then treating this value as tenths
    // of a degree and subtracting off corresponding pitch or roll angle obtained from IMU.
    float errorAngle = Filter::constrainAbs(2*rcCommand, (float)(10*imuConfig.maxAngleInclinationDegrees)) - 
        (float)(10*eulerAnglesDegrees[imuAxis]);

    float PTermAccel = (float)(errorAngle * model->levelP); 

    // Avoid integral windup
    errorAngleI[imuAxis] = Filter::constrainAbs(errorAngleI[imuAxis] + errorAngle, config.angleWindupMax);

    float PTerm = (float)Filter::complementary((float)rcCommand, (float)PTermAccel, prop);

    float ITerm = (float)(ITermGyro * prop);

    float delta = gyroDegreesPerSecond[imuAxis] - lastGyro[imuAxis];
    lastGyro[imuAxis] = gyroDegreesPerSecond[imuAxis];
    float deltaSum = delta1[imuAxis] + delta2[imuAxis] + delta;
    delta2[imuAxis] = delta1[imuAxis];
    delta1[imuAxis] = delta;
    float DTerm = (float)(16*deltaSum * model->ratePitchRollD); // XXX

    return computePid(model->ratePitchRollP, softwareTrim, PTerm, ITerm, DTerm, gyroDegreesPerSecond, imuAxis);
}

void Stabilize::update(float rcCommandRoll, float rcCommandPitch, float rcCommandYaw, 
            float eulerAnglesDegrees[3], float gyroDegreesPerSecond[3])
{
    // Compute proportion of cyclic demand compared to its maximum
    float prop = (std::max)(std::abs(rcCommandRoll), std::abs(rcCommandPitch)) / 0.5f;

    // Pitch, roll use leveling based on Euler angles
    pidRoll  = computePitchRollPid(rcCommandRoll,  model->softwareTrimRoll,  prop, 
            eulerAnglesDegrees, gyroDegreesPerSecond, AXIS_ROLL);
    pidPitch = computePitchRollPid(rcCommandPitch, model->softwareTrimPitch, prop, 
            eulerAnglesDegrees, gyroDegreesPerSecond, AXIS_PITCH);

    // For yaw, P term comes directly from RC command, and D term is zero
    float demandYaw   = 1000 * rcCommandYaw; // XXX
    float ITermGyroYaw = computeITermGyro(model->yawP, model->yawI, demandYaw, gyroDegreesPerSecond, AXIS_YAW);
    pidYaw = computePid(model->yawP, model->softwareTrimYaw, demandYaw, ITermGyroYaw, 0, gyroDegreesPerSecond, AXIS_YAW);

    // Prevent "yaw jump" during yaw correction
    pidYaw = Filter::constrainAbsFloat(pidYaw, 0.1 + std::abs(rcCommandYaw));
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
