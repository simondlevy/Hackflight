/*
   imu.hpp : IMU class header

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/imu.c

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

#include "board.hpp"
#include "config.hpp"

namespace hf {

class IMU {
private:

    ImuConfig   imuConfig;

    Board       * board;

public:

    // Used by Stabilize
    int16_t eulerAngles[3];
    int16_t gyroRaw[3];

    void init(ImuConfig& _imuConfig, Board * _board);
    void update(void);

    // called from Hover
    float computeAccelZ(void);
};


/********************************************* CPP ********************************************************/

void IMU::init(ImuConfig& _imuConfig, Board * _board)
{
    memcpy(&imuConfig, &_imuConfig, sizeof(ImuConfig));
    board = _board;
    board->imuInit();
}

void IMU::update(void)
{
    float eulerAnglesRadians[3];

    // Get raw gyro values from board
    board->imuGetGyro(gyroRaw);

    // Get Euler angles from smoothed accel and raw gyro
    board->imuGetEulerAngles(eulerAnglesRadians);

    // Convert angles from radians to tenths of a degrees
    // NB: roll, pitch in tenths of a degree; yaw in degrees
    eulerAngles[AXIS_ROLL]  = (int16_t)lrintf(eulerAnglesRadians[AXIS_ROLL]  * (1800.0f / M_PI));
    eulerAngles[AXIS_PITCH] = (int16_t)lrintf(eulerAnglesRadians[AXIS_PITCH] * (1800.0f / M_PI));
    eulerAngles[AXIS_YAW]   = (int16_t)(lrintf(eulerAnglesRadians[AXIS_YAW]   * 1800.0f / M_PI) / 10.0f);

    // Convert heading from [-180,+180] to [0,360]
    if (eulerAngles[AXIS_YAW] < 0)
        eulerAngles[AXIS_YAW] += 360;
}

} // namespace hf

