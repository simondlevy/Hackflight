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

    int32_t     accelSum[3];
    int32_t     accelSumCount;
    uint32_t    accelTimeSum;

public:

    void init(void);
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

void IMU::init(void)
{
    for (int k=0; k<3; ++k)
        accelSum[0] = 0;
    accelSumCount = 0;
    accelTimeSum = 0;
}

void IMU::update(uint32_t currentTime, bool armed)
{
    (void)currentTime;
    (void)armed;

    // apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    /*
    accelSum[X] += deadbandFilter((int32_t)lrintf(accel_ned[X]), config.imu.accelXyDeadband);
    accelSum[Y] += deadbandFilter((int32_t)lrintf(accel_ned[Y]), config.imu.accelXyDeadband);
    accelSum[Z] += deadbandFilter((int32_t)lrintf(accz_smooth),  config.imu.accelZDeadband);

    accelTimeSum += dT_usec;
    accelSumCount++;
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

