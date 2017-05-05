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

namespace hf {

class IMU {
private:

    float       accelNed[3];
    int32_t     accelSum[3];
    int32_t     accelSumCount;
    uint32_t    accelTimeSum;
    uint32_t    previousTimeUsec;

    Board       * board;

public:

    // Used by Stabilize
    int16_t eulerAngles[3];
    int16_t gyroRaw[3];

    void init(Board * _board);
    void update(uint32_t currentTime, bool armed);

    // called from Hover
    float computeAccelZ(void);
};


/********************************************* CPP ********************************************************/

/*
static int32_t deadbandFilter(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}
*/

void IMU::init(Board * _board)
{
    board = _board;
    board->imuInit();

    for (int k=0; k<3; ++k)
        accelSum[0] = 0;
    accelSumCount = 0;
    accelTimeSum = 0;
    previousTimeUsec = 0;
}

void IMU::update(uint32_t currentTimeUsec, bool armed)
{
    (void)armed;

    // Track delta time
    uint32_t dT_usec = currentTimeUsec - previousTimeUsec;
    float dT_sec = dT_usec * 1e-6f;
    previousTimeUsec = currentTimeUsec;

    int16_t accelRaw[3];
    float eulerAnglesRadians[3];

    // Get raw acceleromater, gyro values from board
    board->imuReadRaw(accelRaw, gyroRaw);

    // Get Euler angles and raw gyro from board
    board->imuGetEulerAngles(dT_sec, accelRaw, gyroRaw, eulerAnglesRadians);

    // Convert angles from radians to tenths of a degrees
    // NB: roll, pitch in tenths of a degree; yaw in degrees
    eulerAngles[AXIS_ROLL]  = (int16_t)lrintf(eulerAnglesRadians[AXIS_ROLL]  * (1800.0f / M_PI));
    eulerAngles[AXIS_PITCH] = (int16_t)lrintf(eulerAnglesRadians[AXIS_PITCH] * (1800.0f / M_PI));
    eulerAngles[AXIS_YAW]   = (int16_t)(lrintf(eulerAnglesRadians[AXIS_YAW]   * 1800.0f / M_PI) / 10.0f);

    // Convert heading from [-180,+180] to [0,360]
    if (eulerAngles[AXIS_YAW] < 0)
        eulerAngles[AXIS_YAW] += 360;

    // apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    /*
    accelSum[X] += deadbandFilter((int32_t)lrintf(accelNed[X]), config.imu.accelXyDeadband);
    accelSum[Y] += deadbandFilter((int32_t)lrintf(accelNed[Y]), config.imu.accelXyDeadband);
    accelSum[Z] += deadbandFilter((int32_t)lrintf(accelZSmooth),  config.imu.accelZDeadband);

    accelTimeSum += dT_usec;
    accelSumCount++;

    // the accel values have to be rotated into the earth frame
    float rpy[3];
    rpy[0] = -(float)eulerAnglesRadians[AXIS_ROLL];
    rpy[1] = -(float)eulerAnglesRadians[AXIS_PITCH];
    rpy[2] = -(float)eulerAnglesRadians[AXIS_YAW];

    accelNed[X] = accelSmooth[0];
    accelNed[Y] = accelSmooth[1];
    accelNed[Z] = accelSmooth[2];

    rotateV(accelNed, rpy);

    if (!armed) {
        accelZoffset -= accelZoffset / 64;
        accelZoffset += (int32_t)accelNed[Z];
    }
    accelNed[Z] -= accelZoffset / 64;  // compensate for gravitation on z-axis

    accelZSmooth = accelZSmooth + (dT_sec / (fcAcc + dT_sec)) * (accelNed[Z] - accelZSmooth); // low pass filter
    */
}

float IMU::computeAccelZ(void)
{
    float accelZ = 0; //(float)accelSum[2] / (float)accelSumCount * (9.80665f / 10000.0f / config.imu.acc1G);

    accelSum[0] = 0;
    accelSum[1] = 0;
    accelSum[2] = 0;
    accelSumCount = 0;
    accelTimeSum = 0;

    return accelZ;
}


} // namespace hf

