/*
   MPU6050.h : class header for MPU6050 library for Teensy 3.X and Teensy LC

   This file is part of MPU6050.

   MPU6050 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   MPU6050 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with MPU6050.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// Support different boards
#if defined(TEENSYDUINO)
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif

#include <stdint.h>

typedef enum {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
} mpu_gyro_range;

typedef enum {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
} mpu_accel_range;

class MPU6050 {

    public:

        MPU6050(uint8_t address=0x68);

        int begin(mpu_accel_range arange, mpu_gyro_range grange);

        bool getMotion6Counts(int16_t * ax, int16_t * ay, int16_t * az, int16_t * gx, int16_t * gy, int16_t * gz);

    private:

        uint8_t _address;

        void    readAccelData(int16_t * ax, int16_t * ay, int16_t *az);
        void    readGyroData(int16_t * gx, int16_t * gy, int16_t *gz);
        void    writeByte(uint8_t subAddress, uint8_t data);
        uint8_t readByte(uint8_t subAddress);
        void    readBytes(uint8_t subAddress, uint8_t count, uint8_t * dest);
};
