/*
   mpu6000spi.cpp : Experimental class for Invensense MPU6000 IMU using SPI bus

   Copyright (C) 2019 Simon D. Levy 

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

#include "mpu6000spi.h"

MPU6000SPI::MPU6000SPI(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor)
{
    (void)ascale;
    (void)gscale;
    (void)sampleRateDivisor;
}

bool MPU6000SPI::accelReady(void)
{
    return false;
}

bool MPU6000SPI::gyroReady(void)
{
    return false;
}


