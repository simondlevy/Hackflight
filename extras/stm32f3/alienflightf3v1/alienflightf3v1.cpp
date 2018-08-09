/*
   alienflightf3v1.cpp Support MPU6050 IMU on AlienflightF3 V1 board

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

#include "alienflightf3.h"
#include <MPU6050.h>
#include <Wire.h>
#include <debug.hpp>

static const Gscale_t GSCALE = GFS_250DPS;
static const Ascale_t ASCALE = AFS_2G;

void AlienflightF3::init(void)
{
    Wire.begin(2);

    MPU6050 * imu = new MPU6050();

    imu->begin();

    imu->initMPU6050(ASCALE, GSCALE); 

    _imu = imu;
}

bool AlienflightF3::getImu(int16_t accelCount[3], int16_t gyroCount[3])
{
    MPU6050 * imu = (MPU6050 *)_imu;

    if (imu->checkNewData()) {  
        imu->readGyroData(gyroCount);  
        imu->readAccelData(accelCount);  
        return true;
    }

    return false;
}
