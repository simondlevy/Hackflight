/*
   stabilize.cpp : Old-school  PID-based stability class implementation

   Adapted from 

     https://github.com/multiwii/baseflight/blob/master/src/mw.c

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

#ifdef __arm__
extern "C" {
#else
#include <stdio.h>
#endif

#include "hackflight.hpp"
#include "pidvals.hpp"

void Stabilize::init(class RC * _rc, class IMU * _imu)
{
    this->rc = _rc;
    this->imu = _imu;

    for (uint8_t axis=0; axis<3; ++axis) {
        this->lastGyroError[axis] = 0;
        this->delta1[axis] = 0;
        this->delta2[axis] = 0;
    }

    this->rate_p[0] = CONFIG_RATE_PITCHROLL_P;
    this->rate_p[1] = CONFIG_RATE_PITCHROLL_P;
    this->rate_p[2] = CONFIG_YAW_P;

    this->rate_i[0] = CONFIG_RATE_PITCHROLL_I;
    this->rate_i[1] = CONFIG_RATE_PITCHROLL_I;
    this->rate_i[2] = CONFIG_YAW_I;

    this->rate_d[0] = CONFIG_RATE_PITCHROLL_D;
    this->rate_d[1] = CONFIG_RATE_PITCHROLL_D;
    this->rate_d[2] = 0;

    this->resetIntegral();
}

void Stabilize::update(void)
{
    for (uint8_t axis = 0; axis < 3; axis++) {

        int32_t gyroError = this->imu->gyroADC[axis] / 4;

        int32_t error = (int32_t)this->rc->command[axis] * 10 * 8 / 
            this->rate_p[axis] - gyroError;

        int32_t PTermGYRO = this->rc->command[axis];

        this->errorGyroI[axis] = constrain(this->errorGyroI[axis] + error, 
                -16000, +16000); // WindUp

        if ((abs(gyroError) > 640) || ((axis == AXIS_YAW) && 
                    (abs(this->rc->command[axis]) > 100)))
            this->errorGyroI[axis] = 0;

        int32_t ITermGYRO = (this->errorGyroI[axis] / 125 * this->rate_i[axis]) >> 6;

        int32_t PTerm = PTermGYRO;
        int32_t ITerm = ITermGYRO;

        if (axis < 2) {

            // 50 degrees max inclination
            int32_t errorAngle = constrain(2 * this->rc->command[axis], 
                                           -((int)CONFIG_MAX_ANGLE_INCLINATION), 
                                           + CONFIG_MAX_ANGLE_INCLINATION) 
                                 - this->imu->angle[axis];

            int32_t PTermACC = errorAngle * CONFIG_LEVEL_P / 100; 

            this->errorAngleI[axis] = constrain(this->errorAngleI[axis] + errorAngle, 
                    -10000, +10000); // WindUp

            int32_t ITermACC = (this->errorAngleI[axis] * CONFIG_LEVEL_I) >> 12;

            int32_t prop = max(abs(this->rc->command[DEMAND_PITCH]), 
                    abs(this->rc->command[DEMAND_ROLL])); // range [0;500]

            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } 

        // 32 bits is needed for calculation
        PTerm -= gyroError * this->rate_p[axis] / 10 / 8; 

        int32_t delta = gyroError - this->lastGyroError[axis];
        this->lastGyroError[axis] = gyroError;
        int32_t deltaSum = this->delta1[axis] + this->delta2[axis] + delta;
        this->delta2[axis] = this->delta1[axis];
        this->delta1[axis] = delta;
        int32_t DTerm = (deltaSum * this->rate_d[axis]) / 32;
        this->axisPID[axis] = PTerm + ITerm - DTerm;
    }

    // prevent "yaw jump" during yaw correction
    this->axisPID[AXIS_YAW] = constrain(this->axisPID[AXIS_YAW], 
            -100 - abs(this->rc->command[DEMAND_YAW]), 
            +100 + abs(this->rc->command[DEMAND_YAW]));
}

void Stabilize::resetIntegral(void)
{
    this->errorGyroI[AXIS_ROLL] = 0;
    this->errorGyroI[AXIS_PITCH] = 0;
    this->errorGyroI[AXIS_YAW] = 0;
    this->errorAngleI[AXIS_ROLL] = 0;
    this->errorAngleI[AXIS_PITCH] = 0;
}

#ifdef __arm__
} // extern "C"
#endif
