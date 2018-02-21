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
#include "estimators/altitude.hpp"

namespace hf {

    class Ladybug : public RealBoard {

        private:

            static const uint16_t GYRO_RES  = 2000; // degrees per second

            const uint8_t _motorPins[4] = {13, A2, 3, 11};

            const float gyroAdcToRadians = M_PI * (float)GYRO_RES / (1<<15) / 180.;  

            EM7180 _sentral;

            // Altitude-estimation task
            // NB: Try ALT P 50; VEL PID 50;5;30
            // based on https://github.com/betaflight/betaflight/issues/1003 (Glowhead comment at bottom)
            AltitudeEstimator altitudeEstimator = AltitudeEstimator(
                    15,  // Alt P
                    15,  // Vel P
                    15,  // Vel I
                    1);  // Vel D

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

                // Start the EM7180 in master mode, no interrupt
                if (!_sentral.begin()) {
                    while (true) {
                        Serial.println(_sentral.getErrorString());
                    }
                }

                // Initialize the motors
                for (int k=0; k<4; ++k) {
                    analogWriteFrequency(_motorPins[k], 10000);  
                    analogWrite(_motorPins[k], 0);  
                }

                // Initialize the atitude estimator with the accelerator value for 1G
                altitudeEstimator.init(2048);

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

            void getImu(bool armed, float eulerAngles[3], float gyroRates[3])
            {
                _sentral.checkEventStatus();

                if (_sentral.gotError()) {
                    while (true) {
                        Serial.print("ERROR: ");
                        Serial.println(_sentral.getErrorString());
                    }
                }

                if (_sentral.gotQuaternions()) {

                    static float q[4];
                    _sentral.readQuaternions(q);

                    eulerAngles[0] = atan2(2.0f * (q[3] * q[0] + q[1] * q[2]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);
                    eulerAngles[1] = asin(2.0f * (q[0] * q[2] - q[3] * q[1]));
                    eulerAngles[2] = atan2(2.0f * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);   
                }

                if (_sentral.gotGyrometer()) {

                    int16_t gyro[3];

                    _sentral.readGyrometer(gyro);

                    // invert pitch, yaw gyro direction to keep other code simpler
                    gyro[1] = -gyro[1];
                    gyro[2] = -gyro[2];

                    for (uint8_t k=0; k<3; ++k) {
                        gyroRates[k] = gyro[k] * gyroAdcToRadians;
                    }

                    altitudeEstimator.updateGyro(gyroRates, micros());
                }

                if (_sentral.gotAccelerometer()) {

                    int16_t accel[3];
                    _sentral.readAccelerometer(accel);
                    altitudeEstimator.updateAccel(accel, micros());
                }

                if (_sentral.gotBarometer()) {

                    float pressure, temperature;
                    _sentral.readBarometer(pressure, temperature);
                    altitudeEstimator.updateBaro(armed, pressure, micros());
                }

             } // getImu

            void handleAuxSwitch(demands_t & demands)
            { 
                altitudeEstimator.handleAuxSwitch(demands);
            }

            void runPidControllers(demands_t & demands) 
            {
                altitudeEstimator.modifyDemands(demands);
            }

    }; // class Ladybug

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
