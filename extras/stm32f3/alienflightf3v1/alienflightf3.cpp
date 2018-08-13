/*
   alienflightf3.cpp Board class implementation for F3Board

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

#include <f3_board.h>
#include <motor.h>
#include <debug.hpp>
#include <MPU6050.h>
#include <Wire.h>

void F3Board::imuInit(void)
{
    Wire.begin(2);

    MPU6050 * imu = new MPU6050(AFS_2G, GFS_250DPS);

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

bool F3Board::imuRead(void)
{
    MPU6050 * imu = (MPU6050 *)_imu;

    if (imu->checkNewData()) {  

	// Note reversed X/Y order because of IMU rotation
        imu->readAccelerometer(_ay, _ax, _az);
        imu->readGyrometer(_gy, _gx, _gz);

        // Negate for same reason
        _ax = -_ax;
        _gx = -_gx;

        return true;

    }  

    return false;
}

static BrushedMotor * motors[4];

void F3Board::writeMotor(uint8_t index, float value)
{
    motors[index]->writeMicroseconds((uint16_t)(1000*(value+1)));
}

void F3Board::motorInit(void)
{
    // Valid pins for ALIENFLIGHTF3 are 0, 8, 14, 15
    motors[0] = new BrushedMotor(15);
    motors[1] = new BrushedMotor(14);
    motors[2] = new BrushedMotor(8);
    motors[3] = new BrushedMotor(0);
}
