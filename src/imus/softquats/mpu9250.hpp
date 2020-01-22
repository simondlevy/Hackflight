/*
   MPU9250 implementation of softquare-quaternion IMU

   Copyright (c) 2020 Simon D. Levy

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

#include "imus/softquat.hpp"
#include "MPU9250_Master_I2C.h"

namespace hf {

    // Choices are:
    // Gscale: GFS_250 = 250 dps, GFS_500 = 500 dps, GFS_1000 = 1000 dps, GFS_2000DPS = 2000 degrees per second gyro full scale
    // Ascale: AFS_2G = 2 g, AFS_4G = 4 g, AFS_8G = 8 g, and AFS_16G = 16 g accelerometer full scale
    // Mscale: MFS_14BITS = 0.6 mG per LSB and MFS_16BITS = 0.15 mG per LSB
    // Mmode:  Mmode = M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate
    // SAMPLE_RATE_DIVISOR: (1 + SAMPLE_RATE_DIVISOR) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
    // SAMPLE_RATE_DIVISOR = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
    static const MPUIMU::Ascale_t  ASCALE              = MPUIMU::AFS_2G;
    static const MPUIMU::Gscale_t  GSCALE              = MPUIMU::GFS_250DPS;
    static const MPU9250::Mscale_t MSCALE              = MPU9250::MFS_16BITS;
    static const MPU9250::Mmode_t  MMODE               = MPU9250::M_100Hz;
    static const uint8_t           SAMPLE_RATE_DIVISOR = 4;         

    // Instantiate MPU9250 class in master mode
    static MPU9250_Master_I2C _mpu9250_imu(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISOR);

    class MPU9250SoftwareQuaternionIMU : public SoftwareQuaternionIMU {

        private:

            static void error(const char * errmsg) 
            {
                Serial.println(errmsg);
                while (true) ;
            }

        protected:

            virtual void begin(void) override
            {
                // Start the MPU9250
                switch (_mpu9250_imu.begin()) {

                    case MPUIMU::ERROR_IMU_ID:
                        error("Bad IMU device ID");
                    case MPUIMU::ERROR_MAG_ID:
                        error("Bad magnetometer device ID");
                    case MPUIMU::ERROR_SELFTEST:
                        //error("Failed self-test");
                        break;
                }
            }

            virtual bool imuReady(void) override 
            {
                return _mpu9250_imu.checkNewData();
            }

            virtual void imuReadAccelGyro(float & ax, float & ay, float & az, float & gx, float & gy, float &gz) override
            {
                _mpu9250_imu.readAccelerometer(ax, ay, az);
                _mpu9250_imu.readGyrometer(gx, gy, gz);
            }

    }; // class MPU9250SoftwareQuaternionIMU

} // namespace hf
