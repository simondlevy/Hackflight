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

#include "rc.hpp"
#include "config.hpp"
#include "common.hpp"

namespace hf {

class Stabilize {
public:
    int16_t axisPID[3];

    void init(const PidConfig& _pidConfig, const ImuConfig& _imuConfig, Board * _board);

    void update(int16_t rcCommand[4], int16_t gyroADC[3], float eulerAngles[3]);

    void resetIntegral(void);

private:

    uint8_t rate_p[3];
    uint8_t rate_i[3];
    uint8_t rate_d[3];

    int16_t lastGyro[3];
    int32_t delta1[3]; 
    int32_t delta2[3];
    int32_t errorGyroI[3];
    int32_t errorAngleI[2];

    Board * board;

    ImuConfig imuConfig;
    PidConfig pidConfig;
}; 


/********************************************* CPP ********************************************************/

void Stabilize::init(const PidConfig& _pidConfig, const ImuConfig& _imuConfig, Board * _board)
{
    // a hack for debugging
    board = _board;

    // We'll use PID, IMU config values in update() below
    memcpy(&pidConfig, &_pidConfig, sizeof(PidConfig));
    memcpy(&imuConfig, &_imuConfig, sizeof(ImuConfig));

    for (uint8_t axis=0; axis<3; ++axis) {
        lastGyro[axis] = 0;
        delta1[axis] = 0;
        delta2[axis] = 0;
    }

    rate_p[0] = pidConfig.ratePitchrollP;
    rate_p[1] = pidConfig.ratePitchrollP;
    rate_p[2] = pidConfig.yawP;

    rate_i[0] = pidConfig.ratePitchrollI;
    rate_i[1] = pidConfig.ratePitchrollI;
    rate_i[2] = pidConfig.yawI;

    rate_d[0] = pidConfig.ratePitchrollD;
    rate_d[1] = pidConfig.ratePitchrollD;
    rate_d[2] = 0;

    resetIntegral();
}

void Stabilize::update(int16_t rcCommand[4], int16_t gyroADC[3], float eulerAngles[3])
{
    for (uint8_t axis = 0; axis < 3; axis++) {

        int32_t error = ((int32_t)rcCommand[axis] * 10 * 8 / rate_p[axis]) - gyroADC[axis];

        int32_t PTermGyro = rcCommand[axis];

        errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000); // WindUp
        if ((std::abs(gyroADC[axis]) > 640) || ((axis == AXIS_YAW) && (std::abs(rcCommand[axis]) > 100)))
            errorGyroI[axis] = 0;
        int32_t ITermGyro = (errorGyroI[axis] / 125 * rate_i[axis]) >> 6;

        int32_t PTerm = PTermGyro;
        int32_t ITerm = ITermGyro;

        if (axis < 2) {

            // max inclination
            int32_t errorAngle = constrain(2 * rcCommand[axis], 
                - imuConfig.maxAngleInclination, 
                + imuConfig.maxAngleInclination) 
                - 10*eulerAngles[axis];

            int32_t PTermAccel = errorAngle * pidConfig.levelP / 100; 

            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            int32_t ITermAccel = ((int32_t)(errorAngleI[axis] * pidConfig.levelI)) >> 12;

            int32_t prop = (std::max)(std::abs(rcCommand[DEMAND_PITCH]), 
                                    std::abs(rcCommand[DEMAND_ROLL])); // range [0;500]

            PTerm = (PTermAccel * (500 - prop) + PTermGyro * prop) / 500;
            ITerm = (ITermAccel * (500 - prop) + ITermGyro * prop) / 500;
        } 

        PTerm -= (int32_t)gyroADC[axis] * rate_p[axis] / 10 / 8; // 32 bits is needed for calculation

        int32_t delta = gyroADC[axis] - lastGyro[axis];
        lastGyro[axis] = gyroADC[axis];
        int32_t deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        int32_t DTerm = (deltaSum * rate_d[axis]) / 32;
        axisPID[axis] = PTerm + ITerm - DTerm + pidConfig.softwareTrim[axis];
    }

    // prevent "yaw jump" during yaw correction
    axisPID[AXIS_YAW] = constrain(axisPID[AXIS_YAW], 
        -100 - std::abs(rcCommand[DEMAND_YAW]), +100 + std::abs(rcCommand[DEMAND_YAW]));
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
