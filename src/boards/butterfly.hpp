/*
   butterfly.hpp : Implementation of Hackflight Board routines for Butterfly
                   dev board + MPU9250 IMU + MS5637 barometer + brushless motors

   Additional libraries required: https://github.com/simondlevy/MPU9250
                                  https://github.com/simondlevy/MS5637

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

#pragma once

#include <Wire.h>
#include <Servo.h>

#include <MPU9250.h> 

#include "filters.hpp"
#include "hackflight.hpp"
#include "realboard.hpp"

namespace hf {

    // Interrupt support 
    static bool gotNewData;
    static void interruptHandler()
    {
        gotNewData = true;
    }

    class Butterfly : public RealBoard {

        private:

            // Motor pins
            const uint8_t MOTOR_PINS[4] = {3, 4, 5, 6};

            // Min, max PWM values
            const uint16_t PWM_MIN = 990;
            const uint16_t PWM_MAX = 2000;

            // Butterfly board follows Arduino standard for LED pin
            const uint8_t LED_PIN = 13;

            // MPU9250 add-on board has interrupt on Butterfly pin 8
            const uint8_t INTERRUPT_PIN = 8;

            // Paramters to experiment with ------------------------------------------------------------------------

            // MPU9250 full-scale settings
            static const Ascale_t ASCALE = AFS_8G;
            static const Gscale_t GSCALE = GFS_2000DPS;
            static const Mscale_t MSCALE = MFS_16BITS;
            static const Mmode_t  MMODE  = M_100Hz;

            // SAMPLE_RATE_DIVISOR: (1 + SAMPLE_RATE_DIVISOR) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
            // SAMPLE_RATE_DIVISOR = 0 means 1 kHz sample rate for both accel and gyro, 4 means 200 Hz, etc.
            static const uint8_t SAMPLE_RATE_DIVISOR = 0;         

            // Global constants for 6 DoF quaternion filter
            const float GYRO_MEAS_ERROR = M_PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
            const float GYRO_MEAS_DRIFT = M_PI * (0.0f  / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
            const float BETA = sqrtf(3.0f / 4.0f) * GYRO_MEAS_ERROR;   // compute BETA
            const float ZETA = sqrt(3.0f / 4.0f) * GYRO_MEAS_DRIFT;  

            // Update quaternion after this number of gyro updates
            const uint8_t QUATERNION_DIVISOR = 5;

            // Instance variables -----------------------------------------------------------------------------------

            // Use the MPU9250 in pass-through mode
            MPU9250_Passthru _imu = MPU9250_Passthru(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISOR);

            // Run motor ESCs using standard Servo library
            Servo _escs[4];

            // Quaternion support: even though MPU9250 has a magnetometer, we keep it simple for now by 
            // using a 6DOF fiter (accel, gyro)
            MadgwickQuaternionFilter6DOF _quaternionFilter = MadgwickQuaternionFilter6DOF(BETA, ZETA);
            uint8_t _gyroCycleCount;
            float _ax=0,_ay=0,_az=0,_gx=0,_gy=0,_gz=0;

            // Helpers -----------------------------------------------------------------------------------------------

            void error(const char * errmsg) 
            {
                Serial.println(errmsg);
                while (true);
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
                _escs[index].writeMicroseconds((uint16_t)(PWM_MIN+value*(PWM_MAX-PWM_MIN)));
            }

            virtual uint32_t getMicroseconds(void) override
            {
                return micros();
            }

            void delaySeconds(float sec)
            {
                delay((uint32_t)(1000*sec));
            }

            bool getGyrometer(float gyro[3])
            {
                if (gotNewData) {

                    gotNewData = false;

                    if (_imu.checkNewAccelGyroData()) {

                        // Read IMU
                        _imu.readAccelerometer(_ax, _ay, _az);
                        _imu.readGyrometer(_gx, _gy, _gz);

                        // Convert gyrometer values from degrees/sec to radians/sec
                        _gx = radians(_gx);
                        _gy = radians(_gy);
                        _gz = radians(_gz);

                        // Copy gyro values back out
                        gyro[0] = _gx;
                        gyro[1] = _gy;
                        gyro[2] = _gz;

                        // Increment count for quaternion check
                        _gyroCycleCount++;

                        return true;

                    } // if (_imu.checkNewAccelGyroData())

                } // if gotNewData

                return false;
            }

            bool getQuaternion(float quat[4])
            {
                // Update quaternion after some number of IMU readings
                if (_gyroCycleCount == QUATERNION_DIVISOR) {

                    _gyroCycleCount = 0;

                    // Set integration time by time elapsed since last filter update
                    uint32_t timeCurr = micros();
                    static uint32_t _timePrev;
                    float deltat = ((timeCurr - _timePrev)/1000000.0f); 
                    _timePrev = timeCurr;

                    // Run the quaternion on the IMU values acquired in getGyrometer()
                    _quaternionFilter.update(-_ax, _ay, _az, _gx, -_gy, -_gz, deltat);

                    // Copy the quaternion back out
                    quat[0] = _quaternionFilter.q1;
                    quat[1] = _quaternionFilter.q2;
                    quat[2] = _quaternionFilter.q3;
                    quat[3] = _quaternionFilter.q4;

                    return true;
                }

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

                // Set up the interrupt pin, it's set as active high, push-pull
                pinMode(INTERRUPT_PIN, INPUT);
                attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);  

                // Connect to the ESCs and send them the baseline values
                for (uint8_t k=0; k<4; ++k) {
                    _escs[k].attach(MOTOR_PINS[k]);
                    _escs[k].writeMicroseconds(PWM_MIN);
                }

                // Start I^2C
                Wire.begin();
                Wire.setClock(400000); // I2C frequency at 400 kHz

                // Wait a bit
                delay(100);

                // Start the MPU9250
                switch (_imu.begin()) {

                    case MPU_ERROR_IMU_ID:
                        error("Bad IMU device ID");
                    case MPU_ERROR_MAG_ID:
                        error("Bad magnetometer device ID");
                    case MPU_ERROR_SELFTEST:
                        error("Failed self-test");
                    default:
                        break;
                }

                // Initialize the quaternion-update counter
                _gyroCycleCount = 0;

                // Do general real-board initialization
                RealBoard::init();
            }


    }; // class Butterfly

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
