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

// IMU full-scale settings
static const Gscale_t GSCALE = GFS_2000DPS;
static const Ascale_t ASCALE = AFS_8G;

// gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
static const float GyroMeasError = M_PI * (40.0f / 180.0f);     

// first parameter for Madgwick
static const float Beta = sqrt(3.0f / 4.0f) * GyroMeasError;  

// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
static const float GyroMeasDrift = M_PI * (2.0f / 180.0f);      

// second parameter for Madgwick, usually set to a small or zero value
static const float Zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  

static float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer

static float aRes, gRes;

static MPU6050 * imu;

static hf::MadgwickQuaternionFilter6DOF * madgwick;

void F3Board::imuInit(void)
{
    Wire.begin(2);

    imu = new MPU6050();

    madgwick = new hf::MadgwickQuaternionFilter6DOF(Beta, Zeta);

    imu->begin();

    imu->calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  

    imu->initMPU6050(ASCALE, GSCALE);

    aRes = imu->getAres(ASCALE);
    gRes=imu->getGres(GSCALE);
}

bool F3Board::getImu(int16_t accelCount[3], int16_t gyroCount[3])
{
    if (imu->checkNewData()) {  
        imu->readGyroData(gyroCount);  
        imu->readAccelData(accelCount);  
        return true;
    }

    return false;
}
