/*
   butterfly.hpp : Implementation of Hackflight Board routines for Butterfly
                   dev board + MPU9250 IMU

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

#include <Wire.h>
#include <MPU9250.h> 
#include <QuaternionFilters.h>

#include "hackflight.hpp"
#include "realboard.hpp"

namespace hf {

    class Butterfly : public RealBoard {

        private:

            MPU9250  imu;

            const uint8_t LED_PIN       = 13;
            const uint8_t INTERRUPT_PIN = 8;

            const uint8_t Ascale     = AFS_2G;
            const uint8_t Gscale     = GFS_250DPS;
            const uint8_t Mscale     = MFS_16BITS;
            const uint8_t Mmode      = M_100Hz;
            const uint8_t sampleRate = 0x04;         

            float aRes;
            float gRes;
            float mRes;

            // used to read all 14 bytes at once from the MPU9250 accel/gyro
            int16_t imuData[7] = {0,0,0,0,0,0,0};

            // XXX maybe we can declare these in getGyrometer()
            float ax=0, ay=0, az=0;
            float gx=0, gy=0, gz=0;
            float mx=0, my=0, mz=0;

            // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
            const float GyroMeasError = M_PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
            const float GyroMeasDrift = M_PI * (0.0f  / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
            const float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta

            // Quaternion support
            MadgwickQuaternion quaternionCalculator = MadgwickQuaternion(beta);

            float sum = 0;                            // sum for averaging filter update rate
            uint32_t sumCount = 0;                    // used to control display output rate
            uint32_t timePrev = 0;                    // used to calculate integration interval
            float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion

            // XXX we should be loading these values from pre-calibrated data
            float gyroBias[3]        = {0,0,0};
            float accelBias[3]       = {0,0,0};
            float magBias[3]         = {0,0,0};
            float magScale[3]        = {1,1,1};      
            float magCalibration[3]  = {1,1,1};      

        protected:

            void init(void)
            {
                // Begin serial comms
                Serial.begin(115200);

                // Setup LED_PINs and turn them off
                pinMode(LED_PIN, OUTPUT);
                digitalWrite(LED_PIN, LOW);

                // Start I^2C
                Wire.begin();
                Wire.setClock(400000); // I2C frequency at 400 kHz
                delay(1000);

                // Reset the MPU9250
                imu.resetMPU9250(); 

                // get sensor resolutions, only need to do this once
                aRes = imu.getAres(Ascale);
                gRes = imu.getGres(Gscale);
                mRes = imu.getMres(Mscale);

                imu.initMPU9250(Ascale, Gscale, sampleRate); 

                // Get magnetometer calibration from AK8963 ROM
                imu.initAK8963(Mscale, Mmode, magCalibration);

                // Do general real-board initialization
                RealBoard::init();
            }

            void delayMilliseconds(uint32_t msec)
            {
                delay(msec);
            }

            void ledSet(bool is_on)
            { 
                digitalWrite(LED_PIN, is_on ? HIGH : LOW);
            }

            uint8_t serialAvailableBytes(void)
            {
                return Serial.available();
            }

            uint8_t serialReadByte(void)
            {
                return Serial.read();
            }

            void serialWriteByte(uint8_t c)
            {
                Serial.write(c);
            }

            void writeMotor(uint8_t index, float value)
            {
                (void)index;
                (void)value;
            }

            bool getGyrometer(float gyro[3])
            {
                if (imu.checkNewAccelGyroData()) {

                    imu.readMPU9250Data(imuData); 

                    // Convert the accleration value into g's
                    ax = (float)imuData[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
                    ay = (float)imuData[1]*aRes - accelBias[1];   
                    az = (float)imuData[2]*aRes - accelBias[2];  

                    // Convert the gyro value into degrees per second
                    gx = (float)imuData[4]*gRes;  // get actual gyro value, this depends on scale being set
                    gy = (float)imuData[5]*gRes;  
                    gz = (float)imuData[6]*gRes; 

                    if(imu.checkNewMagData()) { // Wait for magnetometer data ready bit to be set

                        int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

                        imu.readMagData(magCount);  // Read the x/y/z adc values

                        // Calculate the magnetometer values in milliGauss
                        // Include factory calibration per data sheet and user environmental corrections
                        // Get actual magnetometer value, this depends on scale being set
                        mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  
                        my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
                        mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  
                        mx *= magScale[0];
                        my *= magScale[1];
                        mz *= magScale[2]; 
                    }

                    // Iterate a fixed number of times per data read cycle, updating the quaternion
                    for (uint8_t i = 0; i < 5; i++) { 

                        uint32_t timeCurr = micros();

                        // Set integration time by time elapsed since last filter update
                        float deltat = ((timeCurr - timePrev)/1000000.0f); 
                        timePrev = timeCurr;

                        sum += deltat; 
                        sumCount++;

                        quaternionCalculator.update(-ax, ay, az, gx*M_PI/180.0f, -gy*M_PI/180.0f, -gz*M_PI/180.0f, my, -mx, mz, deltat, q);
                    }

                    // Copy gyro values back out
                    gyro[0] = gx;
                    gyro[1] = gy;
                    gyro[2] = gz;

                    return true;

                } // if (imu.checkNewAccelGyroData())

                return false;
            }

            bool getQuaternion(float quat[4])
            {
                if(sumCount > 500) {

                    Debug::printf("q1: %+2.2f    q2: %+2.2f    q3: %+2.2f    q4: %+2.2f\n", q[0], q[1], q[2], q[3]);

                    float a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
                    float a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
                    float a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
                    float a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
                    float a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

                    float pitch = -asinf(a32);
                    float roll  = atan2f(a31, a33);
                    float yaw   = atan2f(a12, a22);

                    pitch *= 180.0f / M_PI;
                    roll  *= 180.0f / M_PI;
                    yaw   *= 180.0f / M_PI; 
                    if (yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360

                    Debug::printf("Roll: %+2.2f    Pitch: %+2.2f    Yaw: %+2.2f\n", roll, pitch, yaw);

                    Serial.print("Yaw, Pitch, Roll: ");
                    Serial.print(yaw, 2);
                    Serial.print(", ");
                    Serial.print(pitch, 2);
                    Serial.print(", ");
                    Serial.println(roll, 2);

                    Serial.print("rate = ");
                    Serial.print((float)sumCount/sum, 2);
                    Serial.println(" Hz\n");

                    sumCount = 0;
                    sum = 0;    
                }
                // Fake it for now
                quat[0] = 0.3f;
                quat[1] = 0.0f;
                quat[2] = 0.0f;
                quat[3] = 1.0f;

                return true;
            }

            bool getAccelerometer(float accelGs[3])
            {
                (void)accelGs;
                return false;
            }

            bool getBarometer(float & pressure)
            {
                (void)pressure;
                return false;
            }

    }; // class Butterfly

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
