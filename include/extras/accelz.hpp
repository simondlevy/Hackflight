/*
   accelz.hpp : Support for integrating accelerometer Z axis for estimating altitude

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

#include "config.hpp"
#include "timedtask.hpp"

namespace hf {

class AccelZ {

    private:

        ImuConfig imuConfig;

        int32_t   accelZSum;
        float     accelVelScale;
        int32_t   accelZOffset;
        float     accelZSmooth;
        int16_t   accelSmooth[3];
        uint32_t  accelTimeSum;
        int32_t   accelSumCount;
        float     fcAcc;

        static int32_t deadbandFilter(int32_t value, int32_t deadband);
        static void    rotateV(float v[3], float *delta);

    public:

        void  init(ImuConfig& _imuConfig);
        void  update(int16_t accelRaw[3], float eulerAngles[3], uint32_t currentTimeUsec, bool armed);
        float compute(void);
};

/********************************************* CPP ********************************************************/

void AccelZ::init(ImuConfig& _imuConfig)
{
    memcpy(&imuConfig, &_imuConfig, sizeof(ImuConfig));

    accelVelScale = 9.80665f / imuConfig.accel1G / 10000.0f;

    for (int k=0; k<3; ++k) {
        accelSmooth[k] = 0;
    }
    accelZOffset = 0;
    accelTimeSum = 0;
    accelSumCount = 0;
    accelZSmooth = 0;
    accelZSum = 0;

    // Calculate RC time constant used in the accelZ lpf    
    fcAcc = (float)(0.5f / (M_PI * imuConfig.accelZLpfCutoff)); 
}

void AccelZ::update(int16_t accelRaw[3], float eulerAngles[3], uint32_t currentTimeUsec, bool armed)
{
    // Track delta time
    static uint32_t previousTimeUsec;
    uint32_t dT_usec = currentTimeUsec - previousTimeUsec;
    previousTimeUsec = currentTimeUsec;

    // Rotate accel values into the earth frame

    float rpy[3];
    rpy[0] = -(float)eulerAngles[0];
    rpy[1] = -(float)eulerAngles[1];
    rpy[2] = -(float)eulerAngles[2];

    float       accelNed[3];
    accelNed[0] = accelSmooth[0];
    accelNed[1] = accelSmooth[1];
    accelNed[2] = accelSmooth[2];
    rotateV(accelNed, rpy);

    // Compute vertical acceleration offset at rest
    if (!armed) {
        accelZOffset -= accelZOffset / 64;
        accelZOffset += (int32_t)accelNed[2];
    }

    // Compute smoothed vertical acceleration
    accelNed[2] -= accelZOffset / 64;  // compensate for gravitation on z-axis
    float dT_sec = dT_usec * 1e-6f;
    accelZSmooth = accelZSmooth + (dT_sec / (fcAcc + dT_sec)) * (accelNed[2] - accelZSmooth); // low pass filter

    // Apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    accelZSum += deadbandFilter((int32_t)lrintf(accelZSmooth),  imuConfig.accelZDeadband);

    // Accumulate time and count for integrating accelerometer values
    accelTimeSum += dT_usec;
    accelZSum += deadbandFilter((int32_t)lrintf(accelZSmooth),  imuConfig.accelZDeadband);
}


float AccelZ::compute(void)
{
    // altitude in cm
    static float accelAlt;

    float accZ_tmp = (float)accelZSum / (float)accelSumCount;
    float vel_acc = accZ_tmp * accelVelScale * (float)accelTimeSum;

    // Integrate velocity to get distance (x= a/2 * t^2)
    float dt = accelTimeSum * 1e-6f; // delta acc reading time in seconds
    accelAlt += (vel_acc * 0.5f) * dt;

    // Reset accumulated values
    accelZSum = 0;
    accelTimeSum = 0;
    accelSumCount = 0;

    return accelAlt;
}


void AccelZ::rotateV(float v[3], float *delta)
{
    float * v_tmp = v;

    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(delta[0]);
    sinx = sinf(delta[0]);
    cosy = cosf(delta[1]);
    siny = sinf(delta[1]);
    cosz = cosf(delta[2]);
    sinz = sinf(delta[2]);

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

    v[0] = v_tmp[0] * mat[0][0] + v_tmp[1] * mat[1][0] + v_tmp[2] * mat[2][0];
    v[1] = v_tmp[0] * mat[0][1] + v_tmp[1] * mat[1][1] + v_tmp[2] * mat[2][1];
    v[2] = v_tmp[0] * mat[0][2] + v_tmp[1] * mat[1][2] + v_tmp[2] * mat[2][2];
}


int32_t AccelZ::deadbandFilter(int32_t value, int32_t deadband)
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

} // namespace hf

