/*
   butterfly.hpp : Implementation of Hackflight Board routines for Butterfly
                   dev board + MPU9250 IMU + brushless motors

   Additional library required: https://github.com/simondlevy/KrisWinerMPU9250

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
#include <Servo.h>

#include <MPU9250.h>
#include <ArduinoTransfer.h>

#include "quaternionFilters.hpp"
#include "hackflight.hpp"
#include "realboard.hpp"

namespace hf {

    class Butterfly : public RealBoard {

        private:

            // Create a byte-transfer object for Arduino I^2C
            ArduinoI2C mpu = ArduinoI2C(MPU9250::MPU9250_ADDRESS);
            ArduinoI2C mag = ArduinoI2C(MPU9250::AK8963_ADDRESS);

            // Use the MPU9250 in pass-through mode
            MPU9250Passthru imu = MPU9250Passthru(&mpu, &mag);

            Servo escs[4];

            // Motor pins
            const uint8_t MOTOR_PINS[4] = {3, 4, 5, 6};

            // Min, max PWM values
            const uint16_t PWM_MIN = 990;
            const uint16_t PWM_MAX = 2000;

            // Butterfly board follows Arduino standard for LED pin
            const uint8_t LED_PIN = 13;

            // Paramters to experiment with ------------------------------------------------------------------------

            // Sensor full-scale settings
            const uint8_t Ascale     = AFS_8G;
            const uint8_t Gscale     = GFS_2000DPS;
            const uint8_t Mscale     = MFS_16BITS;
            const uint8_t Mmode      = M_100Hz;

            // sampleRate: (1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so
            // sampleRate = 0 means 1 kHz sample rate for both accel and gyro, 4 means 200 Hz, etc.
            const uint8_t sampleRate = 0x02;

            // Quaternion calculation
            const uint8_t  QuaternionUpdatesPerCycle = 5;    // update quaternion this many times per gyro aquisition
            const uint16_t QuaternionUpdateRate      = 50;   // Hertz

            // Instance variables -----------------------------------------------------------------------------------

            // For scaling to normal units (accelerometer G's, gyrometer rad/sec, magnetometer mGauss)
            float aRes;
            float gRes;
            float mRes;

            // Used to read all 14 bytes at once from the MPU9250 accel/gyro
            int16_t imuData[7] = {0,0,0,0,0,0,0};

            // Global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
            const float GyroMeasError = M_PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
            const float GyroMeasDrift = M_PI * (0.0f  / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
            const float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta

            // Quaternion support
            MadgwickQuaternion quaternionCalculator = MadgwickQuaternion(beta);
            float sum = 0;                                  // sum for averaging filter update rate
            uint32_t sumCount = 0;                          // used to control display output rate
            const uint16_t sumCountMax = 1000 / QuaternionUpdateRate;
            uint32_t timePrev = 0;                          // used to calculate integration interval
            float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};          // vector to hold quaternion

            // XXX we should be loading these values from pre-calibrated data
            float gyroBias[3]        = {0,0,0};
            float accelBias[3]       = {0,0,0};
            float magBias[3]         = {0,0,0};
            float magScale[3]        = {1,1,1};
            float magCalibration[3]  = {1,1,1};

            // Helpers -----------------------------------------------------------------------------------

            // Raw analog-to-digital values converted to radians per second
            float adc2rad(int16_t adc)
            {
                return adc * gRes * M_PI / 180;
            }

        protected:

            void delayMilliseconds(uint32_t msec)
            {
                delay(msec);
            }

            void ledSet(bool is_on)
            {
                digitalWrite(LED_PIN, is_on ? LOW : HIGH);
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
                escs[index].writeMicroseconds((uint16_t)(PWM_MIN+value*(PWM_MAX-PWM_MIN)));
            }

            bool getGyrometer(float gyro[3])
            {
                if (imu.checkNewAccelGyroData()) {

                    imu.readMPU9250Data(imuData);

                    // Convert the accleration value into g's
                    float ax = imuData[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
                    float ay = imuData[1]*aRes - accelBias[1];
                    float az = imuData[2]*aRes - accelBias[2];

                    // Convert the gyro value into degrees per second
                    float gx = adc2rad(imuData[4]);
                    float gy = adc2rad(imuData[5]);
                    float gz = adc2rad(imuData[6]);

                    // We're using pass-through mode, so magnetometer values are updated at their own rate
                    static float mx, my, mz;

                    if (imu.checkNewMagData()) { // Wait for magnetometer data ready bit to be set

                        int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

                        imu.readMagData(magCount);  // Read the x/y/z adc values

                        // Calculate the magnetometer values in milliGauss
                        // Include factory calibration per data sheet and user environmental corrections
                        // Get actual magnetometer value, this depends on scale being set
                        mx = magCount[0]*mRes*magCalibration[0] - magBias[0];
                        my = magCount[1]*mRes*magCalibration[1] - magBias[1];
                        mz = magCount[2]*mRes*magCalibration[2] - magBias[2];
                        mx *= magScale[0];
                        my *= magScale[1];
                        mz *= magScale[2];
                    }

                    // Iterate a fixed number of times per data read cycle, updating the quaternion
                    for (uint8_t i=0; i<QuaternionUpdatesPerCycle; i++) {

                        uint32_t timeCurr = micros();

                        // Set integration time by time elapsed since last filter update
                        float deltat = ((timeCurr - timePrev)/1000000.0f);
                        timePrev = timeCurr;

                        sum += deltat;
                        sumCount++;

                        quaternionCalculator.update(-ax, ay, az, gx, -gy, -gz, my, -mx, mz, deltat, q);
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
                if(sumCount > sumCountMax) {

                    // Reset accumulators
                    sumCount = 0;
                    sum = 0;

                    // Copy quaternion values back out
                    memcpy(quat, q, 4*sizeof(float));

                    return true;
                }

                return false;
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

        public:

            Butterfly(void)
            {
                // Begin serial comms
                Serial.begin(115200);

                // Setup LED pin and turn it off
                pinMode(LED_PIN, OUTPUT);
                digitalWrite(LED_PIN, HIGH);

                // Connect to the ESCs and send them the baseline values
                for (uint8_t k=0; k<4; ++k) {
                    escs[k].attach(MOTOR_PINS[k]);
                    escs[k].writeMicroseconds(PWM_MIN);
                }

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


    }; // class Butterfly

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
