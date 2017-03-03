/*
   imu.hpp : IMU class header

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

enum {
    AXIS_ROLL = 0,
    AXIS_PITCH,
    AXIS_YAW
};

#define CONFIG_ACC_LPF_FACTOR     4
#define CONFIG_ACCZ_DEADBAND      40
#define CONFIG_ACCXY_DEADBAND     40
#define CONFIG_ACCZ_LPF_CUTOFF    5.0F
#define CONFIG_GYRO_CMPF_FACTOR   600    
#define CONFIG_GYRO_CMPFM_FACTOR  250  
#define CONFIG_MORON_THRESHOLD     32

class IMU {

    private:

        int32_t  accelSum[3];
        int32_t  accelSumCount;
        uint32_t accelTimeSum;
        float    accelVelScale;
        uint16_t calibratingGyroCycles;
        uint16_t calibratingAccCycles;
        uint16_t acc1G;
        float    fcAcc;
        float    gyroScale;

    public:

        // shared with other classes
        int16_t  angle[3];      // tenths of a degree
        int16_t  gyroADC[3];    // [-4096,+4096]
        int16_t  accelADC[3];   // [-4096,+4096]

        // called from MW
        void init(uint16_t calibratingGyroCycles, uint16_t calibratingAccCycles);
        void update(uint32_t currentTime, bool armed, uint16_t & calibratingA, uint16_t & calibratingG);

        // called from Hover
        float computeAccelZ(void);
};
