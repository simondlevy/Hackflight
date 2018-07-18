/*
   butterfly.hpp : Implementation of Hackflight Board routines for Butterfly
                   dev board + MPU9250 IMU + MS5637 barometer + brushless motors

   Additional libraries required: https://github.com/simondlevy/MPU9250
                                  https://github.com/simondlevy/MS5637

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
#include <MS5637.h>
#include <ArduinoTransfer.h>

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

            // Create byte-transfer objects for Arduino I^2C 
            ArduinoI2C mpu = ArduinoI2C(MPU9250::MPU9250_ADDRESS);
            ArduinoI2C mag = ArduinoI2C(MPU9250::AK8963_ADDRESS);

            // Use the MPU9250 in pass-through mode
            MPU9250Passthru _imu = MPU9250Passthru(&mpu, &mag);;

            // Use the MS5637 barometer
            MS5637 _baro = MS5637();

            // Run motor ESCs using standard Servo library
            Servo _escs[4];

            // Paramters to experiment with ------------------------------------------------------------------------

            // Sensor full-scale settings
            const Ascale_t ASCALE = AFS_8G;
            const Gscale_t GSCALE = GFS_2000DPS;
            const Mscale_t MSCALE = MFS_16BITS;
            const Mmode_t  MMODE  = M_100Hz;

            // SAMPLE_RATE_DIVISOR: (1 + SAMPLE_RATE_DIVISOR) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
            // SAMPLE_RATE_DIVISOR = 0 means 1 kHz sample rate for both accel and gyro, 4 means 200 Hz, etc.
            const uint8_t SAMPLE_RATE_DIVISOR = 0;         

            // Quaternion calculation
            const uint8_t  QUATERNION_UPDATES_PER_CYCLE = 10;  // update quaternion this many times per gyro aquisition
            const uint16_t QUATERNION_UPDATE_RATE       = 50;   // Hertz

            // Global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
            const float GYRO_MEAS_ERROR = M_PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
            const float GYRO_MES_DRIFT = M_PI * (0.0f  / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
            const float BETA = sqrtf(3.0f / 4.0f) * GYRO_MEAS_ERROR;   // compute BETA

            // These should be computed by running MPU9250/examples/PassthruTest
            const float MAG_BIAS[3]  = {133.f, 399.f, 336.f};
            const float MAG_SCALE[3] = {0.90, 1.02f, 1.11f};

            // Instance variables -----------------------------------------------------------------------------------

            // This will be read from the AK8963 ROM on startup
            float _magCalibration[3]  = {0.f, 0.f, 0.f};

            // For scaling to normal units (accelerometer G's, gyrometer rad/sec, magnetometer mGauss)
            float _aRes;
            float _gRes;
            float _mRes;

            // Used to read all 14 bytes at once from the MPU9250 accel/gyro
            int16_t _imuData[7] = {0,0,0,0,0,0,0};

            // Quaternion support
            MadgwickQuaternionFilter _quaternionFilter = MadgwickQuaternionFilter(BETA);
            uint32_t _sumCount = 0;                          // used to control display output rate
            const uint16_t SUM_COUNT_MAX = 1000 / QUATERNION_UPDATE_RATE;
            uint32_t _timePrev = 0;                          // used to calculate integration interval
            float _q[4] = {1.0f, 0.0f, 0.0f, 0.0f};          // vector to hold quaternion

            // We compute these at startup
            float _gyroBias[3]        = {0,0,0};
            float _accelBias[3]       = {0,0,0};

            // Helpers -----------------------------------------------------------------------------------

            // Raw analog-to-digital values converted to radians per second
            float adc2rad(int16_t adc) 
            {
                return (adc * _gRes) * M_PI / 180;
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

            bool getGyrometer(float gyro[3])
            {
                if (gotNewData) {

                    gotNewData = false;

                    if (_imu.checkNewAccelGyroData()) {

                        _imu.readMPU9250Data(_imuData); 

                        // Convert the accleration value into g's
                        float ax = _imuData[0]*_aRes - _accelBias[0];  // get actual g value, this depends on scale being set
                        float ay = _imuData[1]*_aRes - _accelBias[1];   
                        float az = _imuData[2]*_aRes - _accelBias[2];  

                        // Convert the gyro value into degrees per second
                        float gx = adc2rad(_imuData[4]);
                        float gy = adc2rad(_imuData[5]);
                        float gz = adc2rad(_imuData[6]);

                        // We're using pass-through mode, so magnetometer values are updated at their own rate
                        static float mx, my, mz;

                        if (_imu.checkNewMagData()) { // Wait for magnetometer data ready bit to be set

                            int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

                            _imu.readMagData(magCount);  // Read the x/y/z adc values

                            // Calculate the magnetometer values in milliGauss
                            // Include factory calibration per data sheet and user environmental corrections
                            // Get actual magnetometer value, this depends on scale being set
                            mx = (magCount[0]*_mRes*_magCalibration[0] - MAG_BIAS[0]) * MAG_SCALE[0];  
                            my = (magCount[1]*_mRes*_magCalibration[1] - MAG_BIAS[1]) * MAG_SCALE[1];  
                            mz = (magCount[2]*_mRes*_magCalibration[2] - MAG_BIAS[2]) * MAG_SCALE[2];  
                        }

                        // Iterate a fixed number of times per data read cycle, updating the quaternion
                        for (uint8_t i=0; i<QUATERNION_UPDATES_PER_CYCLE; i++) { 

                            uint32_t timeCurr = micros();

                            // Set integration time by time elapsed since last filter update
                            float deltat = ((timeCurr - _timePrev)/1000000.0f); 
                            _timePrev = timeCurr;

                            _sumCount++;

                            _quaternionFilter.update(-ax, ay, az, gx, -gy, -gz, my, -mx, mz, deltat, _q);
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
                if (_baro.getPressure(&pressure)) {
                    pressure *= 100; // millibars to Pascals
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
                delay(1000);

                // Start the MS5637 barometer
                _baro.begin();

                // Reset the MPU9250
                _imu.resetMPU9250(); 

                // get sensor resolutions, only need to do this once
                _aRes = _imu.getAres(ASCALE);
                _gRes = _imu.getGres(GSCALE);
                _mRes = _imu.getMres(MSCALE);

                // Calibrate gyro and accelerometers, load biases in bias registers
                _imu.calibrateMPU9250(_gyroBias, _accelBias); 

                // Initialize the MPU9250
                _imu.initMPU9250(ASCALE, GSCALE, SAMPLE_RATE_DIVISOR); 

                // Get magnetometer calibration from AK8963 ROM
                _imu.initAK8963(MSCALE, MMODE, _magCalibration);

                // Do general real-board initialization
                RealBoard::init();
            }


    }; // class Butterfly

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
