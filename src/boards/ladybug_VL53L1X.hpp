/*
   ladybug_vl531x.hpp : Ladybug Flight Controller implementation of Hackflight Board routines,
                        with support for VL531X distance sensor

   Uses EM7180 SENtral Sensor Hub in master mode mode

   Additional libraris required: 

       https://github.com/simondlevy/EM7180

       https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library

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
#include <EM7180.h>
#include <SparkFun_VL53L1X_Arduino_Library.h>
#include <stdarg.h>
#include "hackflight.hpp"
#include "realboard.hpp"


namespace hf {

    class Ladybug : public RealBoard {

        private:

            // Current version of LadybugFC uses A4 for LED; but prototype had A1,
            //  and we happen to be using a prototype LadybugFC for this build
            static const uint8_t LED = A1; 

            // Tunable EM7180 parameters
            static const uint8_t  ARES           = 8;    // Gs
            static const uint16_t GRES           = 2000; // degrees per second
            static const uint16_t MRES           = 1000; // microTeslas
            static const uint8_t  MAG_RATE       = 100;  // Hz
            static const uint16_t ACCEL_RATE     = 330;  // Hz
            static const uint16_t GYRO_RATE      = 330;  // Hz
            static const uint8_t  BARO_RATE      = 50;   // Hz
            static const uint8_t  Q_RATE_DIVISOR = 5;    // 1/5 gyro rate

            const uint8_t MOTOR_PINS[4] = {13, A2, 3, 11};

            static constexpr float RANGEFINDER_UPDATE_HZ = 25; // XXX should be using interrupt!

            const uint32_t RANGEFINDER_UPDATE_MICROS = 1e6 / RANGEFINDER_UPDATE_HZ;

            uint32_t _rangefinderMicrosPrev;

            float _gyroAdcToRadians;

            EM7180_Master _sentral = EM7180_Master(ARES, GRES, MRES, MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

            VL53L1X distanceSensor;

            void checkEventStatus(void)
            {
                _sentral.checkEventStatus();

                if (_sentral.gotError()) {
                    while (true) {
                        Serial.print("ERROR: ");
                        Serial.println(_sentral.getErrorString());
                    }
                }
            }

            void delayMilliseconds(uint32_t msec)
            {
                delay(msec);
            }

            uint32_t getMicroseconds(void) 
            {
                return micros();
            }

            void ledSet(bool is_on)
            { 
                digitalWrite(LED, is_on ? HIGH : LOW);
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
                // Scale motor value from [0,1] to [0,255]
                analogWrite(MOTOR_PINS[index], (uint8_t)(value * 255));
            }

            bool getGyrometer(float gyroRates[3])
            {
                // Since gyro is updated most frequently, use it to drive SENtral polling
                checkEventStatus();

                if (_sentral.gotGyrometer()) {

                    int16_t gx, gy, gz;

                    _sentral.readGyrometer(gx, gy, gz);

                    // invert pitch, yaw gyro direction to keep other code simpler
                    gy = -gy;
                    gz = -gz;

                    gyroRates[0] = gx * _gyroAdcToRadians;
                    gyroRates[1] = gy * _gyroAdcToRadians;
                    gyroRates[2] = gz * _gyroAdcToRadians;

                    return true;
                }

                return false;
            }

            bool getRangefinder(float & distance)
            {
                if (distanceSensor.newDataReady()) {
                    uint32_t msec = micros();
                    if (msec-_rangefinderMicrosPrev > RANGEFINDER_UPDATE_MICROS) {
                        distance = distanceSensor.getDistance() / 1000.; // millimeters => meters
                        _rangefinderMicrosPrev = msec; 
                        return true;
                    }
                }

                return false;
            }

            bool getQuaternion(float quat[4])
            {
                if (_sentral.gotQuaternion()) {

                    static float qw, qx, qy, qz;

                    _sentral.readQuaternion(qw, qx, qy, qz);

                    quat[0] = qw;
                    quat[1] = qx;
                    quat[2] = qy;
                    quat[3] = qz;

                    return true;
                }

                return false;
            }

            bool getAccelerometer(float accelGs[3])
            {
                if (_sentral.gotAccelerometer()) {
                    int16_t ax, ay, az;
                    _sentral.readAccelerometer(ax, ay, az);
                    accelGs[0] = ax / 2048.f;
                    accelGs[1] = ay / 2048.f;
                    accelGs[2] = az / 2048.f;
                    return true;
                }

                return false;
            }

            bool getBarometer(float & pressure)
            {
                if (_sentral.gotBarometer()) {
                    float temperature; // ignored
                    _sentral.readBarometer(pressure, temperature);
                    return true;
                }

                return false;
            }

        public:

            Ladybug(void)
            {
                // Begin serial comms
                Serial.begin(115200);

                // Setup LEDs and turn them off
                pinMode(LED, OUTPUT);
                digitalWrite(LED, LOW);

                // Start I^2C
                Wire.begin();

                // Hang a bit before starting up the EM7180
                delay(100);

                // Start the EM7180 in master mode, no interrupt
                if (!_sentral.begin()) {
                    while (true) {
                        Serial.println(_sentral.getErrorString());
                    }
                }

                // Get actual gyro rate for conversion to radians
                uint8_t accFs=0; uint16_t gyroFs=0; uint16_t magFs=0;
                _sentral.getFullScaleRanges(accFs, gyroFs, magFs);
                _gyroAdcToRadians = M_PI * (float)gyroFs / (1<<15) / 180.;  

                // Initialize the motors
                for (int k=0; k<4; ++k) {
                    analogWriteFrequency(MOTOR_PINS[k], 10000);  
                    analogWrite(MOTOR_PINS[k], 0);  
                }

                // Hang a bit more
                delay(100);

                // Initialize the VL53L1X 
                distanceSensor.begin();
                _rangefinderMicrosPrev = 0;


                // Hang a bit more
                delay(100);

                // Do general real-board initialization
                RealBoard::init();
            }

    }; // class Ladybug

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
