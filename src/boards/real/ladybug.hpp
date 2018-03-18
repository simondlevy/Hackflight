/*
   ladybug.hpp : Ladybug Flight Controller implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

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
#include <stdarg.h>
#include "hackflight.hpp"
#include "realboard.hpp"

namespace hf {

    class Ladybug : public RealBoard {

        private:

            const uint8_t _motorPins[4] = {13, A2, 3, 11};

            float gyroAdcToRadians;

            EM7180 _sentral;

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

        protected:

            void init(void)
            {
                // Begin serial comms
                Serial.begin(115200);

                // Setup LEDs and turn them off
                pinMode(A1, OUTPUT);
                digitalWrite(A1, LOW);

                // Start I^2C
                Wire.begin();

                // Hang a bit before starting up the EM7180
                delay(100);

                // Goose up the EM7180 ODRs
                _sentral.accelRate = 330;
                _sentral.gyroRate = 330;
                _sentral.baroRate = 50;
                _sentral.qRateDivisor = 5;

                // Start the EM7180 in master mode, no interrupt
                if (!_sentral.begin()) {
                    while (true) {
                        Serial.println(_sentral.getErrorString());
                    }
                }

                // Get actual gyro rate for conversion to radians
                uint8_t accFs; uint16_t gyroFs; uint16_t magFs;
                _sentral.getFullScaleRanges(accFs, gyroFs, magFs);
                gyroAdcToRadians = M_PI * (float)gyroFs / (1<<15) / 180.;  

                // Initialize the motors
                for (int k=0; k<4; ++k) {
                    analogWriteFrequency(_motorPins[k], 10000);  
                    analogWrite(_motorPins[k], 0);  
                }

                // Hang a bit more
                delay(100);

                // Do general real-board initialization
                RealBoard::init();
            }

            void delayMilliseconds(uint32_t msec)
            {
                delay(msec);
            }

            uint32_t getMicroseconds()
            {
                return micros();
            }

            void ledSet(bool is_on)
            { 
                digitalWrite(A1, is_on ? HIGH : LOW);
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
                uint8_t aval = (uint8_t)(value * 255);

                // Avoid sending the motor the same value over and over
                static uint8_t avalPrev[4];

                if (aval != avalPrev[index]) {
                    analogWrite(_motorPins[index], aval);
                }

                avalPrev[index] = aval;
            }

            bool getGyroRates(float gyroRates[3])
            {
                // Since gyro is updated most frequently, use it to drive SENtral polling
                checkEventStatus();

                if (_sentral.gotGyrometer()) {

                    int16_t gx, gy, gz;

                    _sentral.readGyrometer(gx, gy, gz);

                    // invert pitch, yaw gyro direction to keep other code simpler
                    gy = -gy;
                    gz = -gz;

                    gyroRates[0] = gx * gyroAdcToRadians;
                    gyroRates[1] = gy * gyroAdcToRadians;
                    gyroRates[2] = gz * gyroAdcToRadians;

                    return true;
                }

                return false;
            }

            bool getEulerAngles(float eulerAngles[3])
            {
                if (_sentral.gotQuaternions()) {

                    static float qw, qx, qy, qz;
                    _sentral.readQuaternions(qw, qx, qy, qz);

                    eulerAngles[0] = atan2(2.0f * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz);
                    eulerAngles[1] = asin(2.0f * (qx * qz - qw * qy));
                    eulerAngles[2] = atan2(2.0f * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz); 

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

    }; // class Ladybug

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
