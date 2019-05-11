/*
   MPU6000.cpp : Experimental class for Invensense MPU6000 IMU using SPI bus

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

#include "MPU6000.h"
#include "spi.h"

MPU6000::MPU6000(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor)
{
    _aScale =  ascale;
    _gScale = gscale;
    _sampleRateDivisor = sampleRateDivisor;
}

MPU6000::Error_t MPU6000::begin(void)
{
    if (getId() != MPU_ADDRESS) {
        return ERROR_IMU_ID;
    }

    //if (!selfTest()) {
    //    return ERROR_SELFTEST;
    //}

    writeRegister(PWR_MGMT_1, 0x80);
    delay(100);

    writeRegister(SIGNAL_PATH_RESET, 0x80);
    delay(100);

    writeRegister(PWR_MGMT_1, 0x00);
    delay(100);

    writeRegister(PWR_MGMT_1, INV_CLK_PLL);
    delay(15);

    writeRegister(GYRO_CONFIG, _gScale << 3);
    delay(15);

    writeRegister(ACCEL_CONFIG, _aScale << 3);
    delay(15);

    writeRegister(CONFIG, 0); // no DLPF bits
    delay(15);

    writeRegister(SMPLRT_DIV, _sampleRateDivisor); 
    delay(100);

    // Data ready interrupt configuration
    writeRegister(INT_PIN_CFG, 0x10);  
    delay(15);

    writeRegister(INT_ENABLE, 0x01); 
    delay(15);

    _accelBias[0] = 0;
    _accelBias[1] = 0;
    _accelBias[2] = 0;

    return ERROR_NONE;

}

bool MPU6000::readAccel(int16_t & ax, int16_t & ay, int16_t & az)
{
    ax = 1;
    ay = 2;
    az = 3;

    return true;
}


uint8_t MPU6000::getId()
{
    return readRegister(WHO_AM_I);  
}

uint8_t MPU6000::readRegister(uint8_t subAddress)
{
    uint8_t data;
    readRegisters(subAddress, 1, &data);
    return data;
}

void MPU6000::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    spi_read_registers(subAddress, count, dest);
}

void MPU6000::writeRegister(uint8_t subAddress, uint8_t data)
{
    spi_write_register(subAddress, data);
}
