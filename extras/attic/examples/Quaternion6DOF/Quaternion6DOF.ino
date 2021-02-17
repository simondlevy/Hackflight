/*
   Arduino sketch to test quaternion with MPU6050

   Dependencies: https://github.com/simondlevy/MPU
                 https://github.com/simondlevy/CrossPlatformI2C

   Adapted from https://github.com/kriswiner/MPU6050/blob/master/MPU6050IMU.ino

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

#include <MPU6050.h>

#include <filters.hpp>
#include <sensors/quaternion.hpp>

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

// IMU full-scale settings
static const MPUIMU::Gscale_t GSCALE = MPUIMU::GFS_250DPS;
static const MPUIMU::Ascale_t ASCALE = MPUIMU::AFS_2G;

// gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
static const float GyroMeasError = PI * (40.0f / 180.0f);     

// first parameter for Madgwick
static const float Beta = sqrt(3.0f / 4.0f) * GyroMeasError;  

// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
static const float GyroMeasDrift = PI * (2.0f / 180.0f);      

// second parameter for Madgwick, usually set to a small or zero value
static const float Zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  

static MPU6050 imu(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS);

static hf::MadgwickQuaternionFilter6DOF madgwick(Beta, Zeta);

static void error(const char * errmsg)
{
    while (true) {
        Serial.println(errmsg);
    }
}

static void report(float angle, const char * label)
{
    Serial.print(label);
    Serial.print(":");
    Serial.print(degrees(angle), 2);
    Serial.print(" deg  ");
}

void setup()
{

#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    switch (imu.begin()) {

        case MPUIMU::ERROR_IMU_ID:
            error("Bad device ID");
            break;
        case MPUIMU::ERROR_SELFTEST:
            error("Failed self-test");
            break;
        default:
            break;
    }

    Serial.begin(115200);

}

void loop()
{  
    if (imu.checkNewData()) {  // check if data ready interrupt

        float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
        imu.readAccelerometer(ax, ay, az);
        imu.readGyrometer(gx, gy, gz);

        // Compute deltaT for Madgwick
        uint32_t time = micros();
        static uint32_t _time;
        float deltaT = (time-_time) / 1000000.f;
        _time = time;

        // Use Madgwick to computer quaternion, converting gyro into radians / sec
        madgwick.update(ax, ay, az, radians(gx), radians(gy), radians(gz), deltaT); 

        // Convert quaternion to euler angles
        float q1 = madgwick.q1; 
        float q2 = madgwick.q2; 
        float q3 = madgwick.q3; 
        float q4 = madgwick.q4; 
        float roll  = atan2(2.0f*(q1*q2+q3*q4),q1*q1-q2*q2-q3*q3+q4*q4);
        float pitch =  asin(2.0f*(q2*q4-q1*q3));
        float yaw   = atan2(2.0f*(q2*q3+q1*q4),q1*q1+q2*q2-q3*q3-q4*q4);

        // Report Euler angles in degrees
        report(roll, "Roll");
        report(pitch, "Pitch");
        report(yaw, "Yaw");
    }  
}
