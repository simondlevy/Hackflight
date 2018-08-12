/*
   f3_imu_mpu6050.cpp Support MPU6050 IMU on F3 boards

   Copyright (c) 2018 Simon D. Levy

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

#include "f3_board.h"

#include <hackflight.hpp>
#include <MPU6050.h>
#include <Wire.h>
#include <debug.hpp>
#include <math.h>
#include <string.h>
#include <sensors/quaternion.hpp>

void F3Board::imuInit(void)
{
    Wire.begin(2);
}

bool F3Board::getImu(int16_t accelCount[3], int16_t gyroCount[3])
{
    (void)accelCount;
    (void)gyroCount;

    return false;
}
