/*
   mpu6500.cpp : Ad-hoc implementation of MPU6500 IMU

   Copyright (C) 2018 Simon D. Levy 

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


// Here we put code that interacts with Cleanflight
extern "C" {

#include "mpu6500.h"

enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};


#define SMPLRT_DIV           0x19
#define CONFIG               0x1A
#define GYRO_CONFIG          0x1B
#define ACCEL_CONFIG         0x1C
#define INT_PIN_CFG          0x37
#define INT_ENABLE           0x38
#define ACCEL_XOUT_H         0x3B
#define GYRO_XOUT_H          0x43
#define PWR_MGMT_1           0x6B
#define SIGNAL_PATH_RESET    0x68

    MPU6500::MPU6500(busDevice_t * bus)
    {
        _bus = bus;

        busWriteRegister(_bus, PWR_MGMT_1, 0x80);
        delay(100);
        busWriteRegister(_bus, SIGNAL_PATH_RESET, 0x80);
        delay(100);
        busWriteRegister(_bus, PWR_MGMT_1, 0);
        delay(100);
        busWriteRegister(_bus, PWR_MGMT_1, INV_CLK_PLL);
        delay(15);
        busWriteRegister(_bus, GYRO_CONFIG, INV_FSR_2000DPS << 3);
        delay(15);
        busWriteRegister(_bus, ACCEL_CONFIG, INV_FSR_16G << 3);
        delay(15);
        busWriteRegister(_bus, CONFIG, 0); // no DLPF bits
        delay(15);
        busWriteRegister(_bus, SMPLRT_DIV, 0); 
        delay(100);

        // Data ready interrupt configuration
        busWriteRegister(_bus, INT_PIN_CFG, 0x10);  
        delay(15);

        busWriteRegister(_bus, INT_ENABLE, 0x01); 
        delay(15);
    }

    bool MPU6500::read(int16_t & ax, int16_t & ay, int16_t & az, int16_t & gx, int16_t & gy, int16_t & gz)
    {
        /*
        static const uint8_t dataToSend[7] = {GYRO_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        uint8_t data[7];
        if (!spiBusTransfer(_bus, dataToSend, data, 7)) {
            return false;
        }

        gy = (int16_t)((data[1] << 8) | data[2]);
        gx = (int16_t)((data[3] << 8) | data[4]);
        gz = (int16_t)((data[5] << 8) | data[6]);
        */

        uint8_t data[6];
        
        if (!spiBusReadRegisterBuffer(_bus, GYRO_XOUT_H | 0x80, data, 6)) {
            return false;
        }

        gx = (int16_t)((data[0] << 8) | data[1]);
        gy = (int16_t)((data[2] << 8) | data[3]);
        gz = (int16_t)((data[4] << 8) | data[5]);

        spiBusReadRegisterBuffer(_bus, ACCEL_XOUT_H | 0x80, data, 6);

        ax = (int16_t)((data[0] << 8) | data[1]);
        ay = (int16_t)((data[2] << 8) | data[3]);
        az = (int16_t)((data[4] << 8) | data[5]);

        return true;
    }

} // extern "C"
