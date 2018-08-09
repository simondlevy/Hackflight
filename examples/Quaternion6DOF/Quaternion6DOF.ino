/*
   Quaternion6DOF.ino : Arduino sketch to test quaternion with MPU6050

   Dependencies: https://github.com/simondlevy/MPU6050
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
static const Gscale_t GSCALE = GFS_250DPS;
static const Ascale_t ASCALE = AFS_2G;

// gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
static const float GyroMeasError = PI * (40.0f / 180.0f);     

// first parameter for Madgwick
static const float Beta = sqrt(3.0f / 4.0f) * GyroMeasError;  

// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
static const float GyroMeasDrift = PI * (2.0f / 180.0f);      

// second parameter for Madgwick, usually set to a small or zero value
static const float Zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  

static float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer

static float aRes, gRes;

static MPU6050 mpu;

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion

static hf::MadgwickQuaternionFilter6DOF madgwick(Beta, Zeta);

void setup()
{

#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    mpu.begin();

    Serial.begin(115200);

    Serial.println("MPU6050");
    Serial.println("6-DOF 16-bit");
    Serial.println("motion sensor");
    Serial.println("60 ug LSB");

    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t c = mpu.getMPU6050ID();
    Serial.print("I AM ");
    Serial.print(c, HEX);  
    Serial.print(" I Should Be ");
    Serial.println(0x68, HEX); 

    if (c == 0x68) // WHO_AM_I should always be 0x68
    {  
        Serial.println("MPU6050 is online...");

        float SelfTest[6];               // Gyro and accelerometer self-test sensor output
        mpu.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
        Serial.print("x-axis self test: acceleration trim within : ");
        Serial.print(SelfTest[0],1);
        Serial.println("% of factory value");
        Serial.print("y-axis self test: acceleration trim within : ");
        Serial.print(SelfTest[1],1);
        Serial.println("% of factory value");
        Serial.print("z-axis self test: acceleration trim within : ");
        Serial.print(SelfTest[2],1);
        Serial.println("% of factory value");
        Serial.print("x-axis self test: gyration trim within : ");
        Serial.print(SelfTest[3],1);
        Serial.println("% of factory value");
        Serial.print("y-axis self test: gyration trim within : ");
        Serial.print(SelfTest[4],1);
        Serial.println("% of factory value");
        Serial.print("z-axis self test: gyration trim within : ");
        Serial.print(SelfTest[5],1);
        Serial.println("% of factory value");

        if (SelfTest[0] < 1.0f && 
                SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && 
                SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && 
                SelfTest[5] < 1.0f) {

            Serial.println("Pass Selftest!");  

            mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
            mpu.initMPU6050(ASCALE, GSCALE);
            Serial.println("MPU6050 initialized for active data mode...."); 

            aRes = mpu.getAres(ASCALE);
            gRes=mpu.getGres(GSCALE);
        }
        else
        {
            Serial.print("Could not connect to MPU6050: 0x");
            Serial.println(c, HEX);
            while (true) ; // Loop forever if communication doesn't happen
        }
    }
}

void loop()
{  
    if (mpu.checkNewData()) {  // check if data ready interrupt

        int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output

        mpu.readAccelData(accelCount);  // Read the x/y/z adc values

        // Now we'll calculate the accleration value into actual g's
        float ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
        float ay = (float)accelCount[1]*aRes - accelBias[1];   
        float az = (float)accelCount[2]*aRes - accelBias[2];  

        int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output

        mpu.readGyroData(gyroCount);  // Read the x/y/z adc values

        // Calculate the gyro value into actual degrees per second
        float gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
        float gy = (float)gyroCount[1]*gRes - gyroBias[1];  
        float gz = (float)gyroCount[2]*gRes - gyroBias[2];   

        // Compute deltaT for Madgwick
        uint32_t time = micros();
        static uint32_t _time;
        float deltaT = (time-_time) / 1000000.f;
        _time = time;

        // Use Madgwick to computer quaternion, converting gyro into radians / sec
        madgwick.update(ax, ay, az, radians(gx), radians(gy), radians(gz), deltaT, q);

        // Convert quaternion to euler angles
        float euler[3];
        hf::Quaternion::computeEulerAngles(q, euler);

        // Report Euler angles in degrees
        Serial.print("Roll: ");
        Serial.print(degrees(euler[0]), 2);
        Serial.print(" deg  ");
        Serial.print("Pitch: ");
        Serial.print(degrees(euler[1]), 2);
        Serial.print(" deg  ");
        Serial.print("Yaw: ");
        Serial.print(degrees(euler[2]), 2);
        Serial.println(" deg");
    }  
}
