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

            // Sensor full-scale settings
            static const Ascale_t ASCALE = AFS_8G;
            static const Gscale_t GSCALE = GFS_2000DPS;
            static const Mscale_t MSCALE = MFS_16BITS;
            static const Mmode_t  MMODE  = M_100Hz;

            // SAMPLE_RATE_DIVISOR: (1 + SAMPLE_RATE_DIVISOR) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
            // SAMPLE_RATE_DIVISOR = 0 means 1 kHz sample rate for both accel and gyro, 4 means 200 Hz, etc.
            static const uint8_t SAMPLE_RATE_DIVISOR = 0;         

            // Quaternion calculation
            static const uint8_t  QUATERNION_UPDATES_PER_CYCLE = 10;  // update quaternion this many times per gyro aquisition
            static const uint16_t QUATERNION_UPDATE_RATE       = 50;   // Hertz

            // Global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
            const float GYRO_MEAS_ERROR = M_PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
            const float GYRO_MEAS_DRIFT = M_PI * (0.0f  / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
            const float BETA = sqrtf(3.0f / 4.0f) * GYRO_MEAS_ERROR;   // compute BETA
            const float ZETA = sqrt(3.0f / 4.0f) * GYRO_MEAS_DRIFT;  


            // Instance variables -----------------------------------------------------------------------------------

            // Use the MPU9250 in pass-through mode
            MPU9250_Passthru _imu = MPU9250_Passthru(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISOR);

            // Run motor ESCs using standard Servo library
            Servo _escs[4];

            // Quaternion support
            MadgwickQuaternionFilter9DOF _quaternionFilter = MadgwickQuaternionFilter9DOF(BETA);
            uint32_t _sumCount = 0;                          // used to control display output rate
            const uint16_t SUM_COUNT_MAX = 1000 / QUATERNION_UPDATE_RATE;
            uint32_t _timePrev = 0;                          // used to calculate integration interval
            float _q[4] = {1.0f, 0.0f, 0.0f, 0.0f};          // vector to hold quaternion

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

                        float ax=0, ay=0, az=0;
                        _imu.readAccelerometer(ax, ay, az);

                        float gx=0, gy=0, gz=0;
                        _imu.readGyrometer(gx, gy, gz);

                        gx = radians(gx);
                        gy = radians(gy);
                        gz = radians(gz);

                        // We're using pass-through mode, so magnetometer values are updated at their own rate
                        static float mx, my, mz;

                        if (_imu.checkNewMagData()) { // Wait for magnetometer data ready bit to be set

                            _imu.readMagnetometer(mx, my, mz);
                        }

                        // Iterate a fixed number of times per data read cycle, updating the quaternion
                        for (uint8_t i=0; i<QUATERNION_UPDATES_PER_CYCLE; i++) { 

                            uint32_t timeCurr = micros();

                            // Set integration time by time elapsed since last filter update
                            float deltat = ((timeCurr - _timePrev)/1000000.0f); 
                            _timePrev = timeCurr;

                            _sumCount++;

                            _quaternionFilter.update(-ax, ay, az, gx, -gy, -gz, my, -mx, mz, deltat, _q);

                            Debug::printf("%+3.3f %+3.3f %+3.3f %+3.3f\n", _q[0], _q[1], _q[2], _q[3]);
                        }

                        // Copy gyro values back out
                        gyro[0] = gx;
                        gyro[1] = gy;
                        gyro[2] = gz;

                        return true;

                    } // if (_imu.checkNewAccelGyroData())

                } // if gotNewData

                return false;
            }

            bool getQuaternion(float quat[4])
            {
                if(_sumCount > SUM_COUNT_MAX) {

                    // Reset accumulators
                    _sumCount = 0;

                    // Copy quaternion values back out
                    memcpy(quat, _q, 4*sizeof(float));

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
                delay(1000);

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
                        Serial.println("MPU6050 online!\n");
                }


                // Do general real-board initialization
                RealBoard::init();
            }


    }; // class Butterfly

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
