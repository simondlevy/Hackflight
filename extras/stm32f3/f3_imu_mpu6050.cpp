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

static const Ascale_t ASCALE = AFS_2G;
static const Gscale_t GSCALE = GFS_250DPS;

void F3Board::imuInit(void)
{
    Wire.begin(2);

    MPU6050 * imu = new MPU6050(ASCALE, GSCALE);

    switch (imu->begin()) {

        case MPU_ERROR_ID:
            error("Bad device ID");
            break;
        case MPU_ERROR_SELFTEST:
            error("Failed self-test");
            break;
        default:
            break;
    }
 
    _imu = imu;
}

bool F3Board::getGyrometer(float gyroRates[3]) 
{
    MPU6050 * imu = (MPU6050 *)_imu;

    if (imu->checkNewData()) {  

        imu->readAccelerometer(_ax, _ay, _az);

        imu->readGyrometer(_gx, _gy, _gz);

        hf::Debug::printf("gotGyro!\n");
    }  

    return false;
}
