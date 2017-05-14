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

// Used by MW32
enum {
    X = 0,
    Y,
    Z
};

namespace hf {

class IMU {
private:

    float       accelLpf[3];
    int16_t     accelSmooth[3];
    float       accelNed[3];
    int32_t     accelSum[3];
    int32_t     accelSumCount;
    uint32_t    accelTimeSum;
    int32_t     accelZOffset;
    float       accelZSmooth;
    float       fcAcc;
    uint32_t    previousTimeUsec;

    ImuConfig   imuConfig;

    Board       * board;

    static int32_t deadbandFilter(int32_t value, int32_t deadband);

public:

    // Used by Stabilize
    int16_t eulerAngles[3];
    int16_t gyroRaw[3];

    // Used by MW32
    static void rotateV(float v[3], float *delta);

    void init(ImuConfig& _imuConfig, Board * _board);
    void update(uint32_t currentTime, bool armed);

    // called from Hover
    float computeAccelZ(void);
};


/********************************************* CPP ********************************************************/

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void IMU::rotateV(float v[3], float *delta)
{
    float * v_tmp = v;

    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(delta[AXIS_ROLL]);
    sinx = sinf(delta[AXIS_ROLL]);
    cosy = cosf(delta[AXIS_PITCH]);
    siny = sinf(delta[AXIS_PITCH]);
    cosz = cosf(delta[AXIS_YAW]);
    sinz = sinf(delta[AXIS_YAW]);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    v[X] = v_tmp[X] * mat[0][0] + v_tmp[Y] * mat[1][0] + v_tmp[Z] * mat[2][0];
    v[Y] = v_tmp[X] * mat[0][1] + v_tmp[Y] * mat[1][1] + v_tmp[Z] * mat[2][1];
    v[Z] = v_tmp[X] * mat[0][2] + v_tmp[Y] * mat[1][2] + v_tmp[Z] * mat[2][2];
}


int32_t IMU::deadbandFilter(int32_t value, int32_t deadband)
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

void IMU::init(ImuConfig& _imuConfig, Board * _board)
{
    memcpy(&imuConfig, &_imuConfig, sizeof(ImuConfig));
    board = _board;
    board->imuInit();

    for (int k=0; k<3; ++k) {
        accelSum[0] = 0;
        accelLpf[k] = 0;
        accelSmooth[k] = 0;
    }
    accelSumCount = 0;
    accelTimeSum = 0;
    accelZOffset = 0;
    accelZSmooth = 0;
    previousTimeUsec = 0;

    // Calculate RC time constant used in the accelZ lpf    
    fcAcc = (float)(0.5f / (M_PI * imuConfig.accelzLpfCutoff)); 
}

void IMU::update(uint32_t currentTimeUsec, bool armed)
{
    // Track delta time
    uint32_t dT_usec = currentTimeUsec - previousTimeUsec;
    float dT_sec = dT_usec * 1e-6f;
    previousTimeUsec = currentTimeUsec;

    int16_t accelRaw[3];
    float eulerAnglesRadians[3];

    // Get raw acceleromater, gyro values from board
    board->imuReadRaw(accelRaw, gyroRaw);

    // Calibrate raw values
    board->imuCalibrate(accelRaw, gyroRaw);

    // Smoothe the accelerometer values
    for (uint8_t axis = 0; axis < 3; axis++) {
        if (imuConfig.accelLpfFactor > 0) {
            accelLpf[axis] = accelLpf[axis] * (1.0f - (1.0f / imuConfig.accelLpfFactor)) + 
                accelRaw[axis] * (1.0f / imuConfig.accelLpfFactor);
            accelSmooth[axis] = (int16_t)accelLpf[axis];
        } else {
            accelSmooth[axis] = accelRaw[axis];
        }
    }

    // Get Euler angles from smoothed accel and raw gyro
    board->imuGetEulerAngles(dT_sec, accelSmooth, gyroRaw, eulerAnglesRadians);

    // Convert angles from radians to tenths of a degrees
    // NB: roll, pitch in tenths of a degree; yaw in degrees
    eulerAngles[AXIS_ROLL]  = (int16_t)lrintf(eulerAnglesRadians[AXIS_ROLL]  * (1800.0f / M_PI));
    eulerAngles[AXIS_PITCH] = (int16_t)lrintf(eulerAnglesRadians[AXIS_PITCH] * (1800.0f / M_PI));
    eulerAngles[AXIS_YAW]   = (int16_t)(lrintf(eulerAnglesRadians[AXIS_YAW]   * 1800.0f / M_PI) / 10.0f);

    // Convert heading from [-180,+180] to [0,360]
    if (eulerAngles[AXIS_YAW] < 0)
        eulerAngles[AXIS_YAW] += 360;

    // Rotate accel values into the earth frame
    float rpy[3];
    rpy[X] = -(float)eulerAnglesRadians[AXIS_ROLL];
    rpy[Y] = -(float)eulerAnglesRadians[AXIS_PITCH];
    rpy[Z] = -(float)eulerAnglesRadians[AXIS_YAW];
    accelNed[X] = accelSmooth[X];
    accelNed[Y] = accelSmooth[Y];
    accelNed[Z] = accelSmooth[Z];
    rotateV(accelNed, rpy);

    // Compute vertical acceleration offset at rest
    if (!armed) {
        accelZOffset -= accelZOffset / 64;
        accelZOffset += (int32_t)accelNed[Z];
    }

    // Compute smoothed vertical acceleration
    accelNed[Z] -= accelZOffset / 64;  // compensate for gravitation on z-axis
    accelZSmooth = accelZSmooth + (dT_sec / (fcAcc + dT_sec)) * (accelNed[Z] - accelZSmooth); // low pass filter

    // Apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    accelSum[X] += deadbandFilter((int32_t)lrintf(accelNed[X]), imuConfig.accelXyDeadband);
    accelSum[Y] += deadbandFilter((int32_t)lrintf(accelNed[Y]), imuConfig.accelXyDeadband);
    accelSum[Z] += deadbandFilter((int32_t)lrintf(accelZSmooth),  imuConfig.accelZDeadband);

    // Accumulate time and count for integrating accelerometer values
    accelTimeSum += dT_usec;
    accelSumCount++;
}

float IMU::computeAccelZ(void)
{
    float accelZ = (float)accelSum[Z] / (float)accelSumCount * (9.80665f / 10000.0f / imuConfig.accel1G);

    accelSum[0] = 0;
    accelSum[1] = 0;
    accelSum[2] = 0;
    accelSumCount = 0;
    accelTimeSum = 0;

    return accelZ;
}

} // namespace hf

