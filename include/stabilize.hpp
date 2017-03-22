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
#include "imu.hpp"
#include "config.hpp"
#include "common.hpp"

namespace hf {

class Stabilize {
public:
    int16_t axisPID[3];

    void init(const PidConfig& config, RC * _rc, IMU * _imu);

    void update(void);

    void resetIntegral(void);

private:

    RC  * rc;
    IMU * imu;

    uint8_t rate_p[3];
    uint8_t rate_i[3];
    uint8_t rate_d[3];

    int16_t lastGyro[3];
    int32_t delta1[3]; 
    int32_t delta2[3];
    int32_t errorGyroI[3];
    int32_t errorAngleI[2];

    PidConfig config;
}; 


/********************************************* CPP ********************************************************/

void Stabilize::init(const PidConfig& pidConfig, RC * _rc, IMU * _imu)
{
    memcpy(&config, &pidConfig, sizeof(PidConfig));

    rc = _rc;
    imu = _imu;

    for (uint8_t axis=0; axis<3; ++axis) {
        lastGyro[axis] = 0;
        delta1[axis] = 0;
        delta2[axis] = 0;
    }

    rate_p[0] = config.ratePitchrollP;
    rate_p[1] = config.ratePitchrollP;
    rate_p[2] = config.yawP;

    rate_i[0] = config.ratePitchrollI;
    rate_i[1] = config.ratePitchrollI;
    rate_i[2] = config.yawI;

    rate_d[0] = config.ratePitchrollD;
    rate_d[1] = config.ratePitchrollD;
    rate_d[2] = 0;

    resetIntegral();
}

void Stabilize::update(void)
{
    for (uint8_t axis = 0; axis < 3; axis++) {

        int32_t error = (int32_t)rc->command[axis] * 10 * 8 / rate_p[axis];
        error -= imu->gyroADC[axis];

        int32_t PTermGYRO = rc->command[axis];

        errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000); // WindUp
        if ((std::abs(imu->gyroADC[axis]) > 640) || ((axis == AXIS_YAW) && (std::abs(rc->command[axis]) > 100)))
            errorGyroI[axis] = 0;
        int32_t ITermGYRO = (errorGyroI[axis] / 125 * rate_i[axis]) >> 6;

        int32_t PTerm = PTermGYRO;
        int32_t ITerm = ITermGYRO;

        if (axis < 2) {

            // max inclination
            int32_t errorAngle = constrain(2 * rc->command[axis], 
                -imu->config.maxAngleInclination, 
                + imu->config.maxAngleInclination) 
                - imu->angle[axis];

            int32_t PTermACC = errorAngle * config.levelP / 100; 

            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            int32_t ITermACC = ((int32_t)(errorAngleI[axis] * config.levelI)) >> 12;

            int32_t prop = (std::max)(std::abs(rc->command[DEMAND_PITCH]), 
                                    std::abs(rc->command[DEMAND_ROLL])); // range [0;500]

            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } 

        PTerm -= (int32_t)imu->gyroADC[axis] * rate_p[axis] / 10 / 8; // 32 bits is needed for calculation
        int32_t delta = imu->gyroADC[axis] - lastGyro[axis];
        lastGyro[axis] = imu->gyroADC[axis];
        int32_t deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        int32_t DTerm = (deltaSum * rate_d[axis]) / 32;
        axisPID[axis] = PTerm + ITerm - DTerm;
    }

    // prevent "yaw jump" during yaw correction
    axisPID[AXIS_YAW] = constrain(axisPID[AXIS_YAW], 
        -100 - std::abs(rc->command[DEMAND_YAW]), +100 + std::abs(rc->command[DEMAND_YAW]));
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
