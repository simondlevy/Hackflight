/*
   MPU6050.cpp : implementation of MPU6050 library for Teensy 3.X and Teensy LC

   Code adapted from:

       https://github.com/kriswiner/MPU-6050/blob/master/MPU6050BasicExample.ino
       https://github.com/bolderflight/MPU9250

   This file is part of MPU6050.

   MPU6050 is free software: you can redistribute it and/or modify
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

extern "C" {

#include "MPU6050.h"

#include <bus_i2c.h>

// Define registers per MPU6050, Register Map and Descriptions, Rev 4.2, 08/19/2013 6 DOF Motion sensor fusion device
// Invensense Inc., www.invensense.com
// See also MPU-6050 Register Map and Descriptions, Revision 4.0, RM-MPU-6050A-00, 9/12/2012 for registers not listed in 
// above document; the MPU6050 and MPU-9150 are virtually identical but the latter has an on-board magnetic sensor

#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68

MPU6050::MPU6050(uint8_t address)
{
    _address = address;
}

int MPU6050::begin(mpu_accel_range arange, mpu_gyro_range grange)
{  
    uint8_t c = MPU6050::readByte(WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050

    if (c != 0x68) // WHO_AM_I should always be 0x68
        return -1;

    // get stable time source
    writeByte(PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    writeByte(CONFIG, 0x03);  

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(SMPLRT_DIV, 0x04);  // Use a 200 Hz sample rate 

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    c =  readByte(GYRO_CONFIG);
    writeByte(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    writeByte(GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeByte(GYRO_CONFIG, c | grange << 3); // Set full scale range for the gyro

    // Set accelerometer configuration
    c =  readByte(ACCEL_CONFIG);
    writeByte(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    writeByte(ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeByte(ACCEL_CONFIG, c | arange << 3); // Set full scale range for the accelerometer 
    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(INT_PIN_CFG, 0x02);    
    writeByte(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

    // Success!
    return 0;
}

bool MPU6050::getMotion6Counts(int16_t * ax, int16_t * ay, int16_t * az, int16_t * gx, int16_t * gy, int16_t * gz)
{
    // If data ready bit set, all data registers have new data
    if (readByte(INT_STATUS) & 0x01) {  // check if data ready interrupt

        readAccelData(ax, ay, az);  // Read the x/y/z adc values
        readGyroData(gx, gy, gz);  // Read the x/y/z adc values

        return true;
    } 

    return false;
}

void MPU6050::readAccelData(int16_t * ax, int16_t * ay, int16_t *az)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    *ax = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    *ay = (int16_t)((rawData[2] << 8) | rawData[3]) ;  
    *az = (int16_t)((rawData[4] << 8) | rawData[5]) ; 
}

void MPU6050::readGyroData(int16_t * gx, int16_t * gy, int16_t * gz)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    *gx = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    *gy = (int16_t)((rawData[2] << 8) | rawData[3]) ;  
    *gz = (int16_t)((rawData[4] << 8) | rawData[5]) ; 
}

void MPU6050::writeByte(uint8_t subAddress, uint8_t data)
{
    i2cWrite(0x68, subAddress, data);
}

uint8_t MPU6050::readByte(uint8_t subAddress)
{
    uint8_t byte;
    readBytes(subAddress, 1, &byte);
    return byte;
}

void MPU6050::readBytes(uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    i2cRead(0x68, subAddress, count, dest);
}

} // extern "C"
