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
#include "estimators/altitude_estimator.hpp"

namespace hf {

    class Ladybug : public RealBoard {

        private:

            static const uint8_t  ACCEL_RES = 8;    // Gs
            static const uint16_t GYRO_RES  = 2000; // degrees per second
            static const uint16_t MAG_RES   = 2000; // Tesla

            uint8_t _motorPins[4] = {13, A2, 3, 11};

            float _eulerAnglesRadians[3];

            EM7180 _sentral;

            // XXX We should be getting altitude estimation from EM7180
            AltitudeEstimator   altitudeEstimator;
            Timer altitudeTimer   = Timer(40);

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

                // Start the EM7180
                uint8_t status = _sentral.begin(ACCEL_RES, GYRO_RES, MAG_RES);
                while (status) {
                    Serial.println(EM7180::errorToString(status));
                }

                // Initialize the motors
                for (int k=0; k<4; ++k) {
                    analogWriteFrequency(_motorPins[k], 10000);  
                    analogWrite(_motorPins[k], 0);  
                }

                // Initialize the atitude estimator
                altitudeEstimator.init();

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

            void getState(vehicle_state_t & state)
            {
                uint8_t errorStatus = _sentral.poll();

                if (errorStatus) {
                    Serial.print("ERROR: ");
                    Serial.println(EM7180::errorToString(errorStatus));
                    return;
                }

                static float q[4];
                _sentral.getQuaternions(q);

                float yaw   = atan2(2.0f * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);   
                float pitch = -asin(2.0f * (q[0] * q[2] - q[3] * q[1]));
                float roll  = atan2(2.0f * (q[3] * q[0] + q[1] * q[2]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);

                // Also store Euler angles for extrasUpdateAccelZ()
                state.pose.orientation[0].value = _eulerAnglesRadians[0] = roll;
                state.pose.orientation[1].value = _eulerAnglesRadians[1] = -pitch; // compensate for IMU orientation
                state.pose.orientation[2].value = _eulerAnglesRadians[2] = yaw;

                int16_t gyroRaw[3];

                _sentral.getGyroRaw(gyroRaw[0], gyroRaw[1], gyroRaw[2]);

                gyroRaw[1] = -gyroRaw[1];
                gyroRaw[2] = -gyroRaw[2];

                for (uint8_t k=0; k<3; ++k) {
                    float gyroDegrees = (float)GYRO_RES * gyroRaw[k] / (1<<15); // raw to degrees
                    state.pose.orientation[k].deriv = M_PI * gyroDegrees / 180.;  // degrees to radians
                }

                // Fuse altitude estimator with accelerometer data
                int16_t accelRaw[3];
                _sentral.getAccelRaw(accelRaw[0], accelRaw[1], accelRaw[2]);
                float accelGs[3];
                for (uint8_t k=0; k<3; ++k) {
                    accelGs[k] = (accelRaw[k]-2048.) / (1<<15) * ACCEL_RES + 1;
                }
                altitudeEstimator.fuseWithImu(state, accelGs, micros());

            } // getState

            void runEstimators(vehicle_state_t & state, uint32_t currentTime)
            {
                if (altitudeTimer.checkAndUpdate(currentTime)) {
                    float pressure, temperature;
                    _sentral.getBaro(pressure, temperature);
                    altitudeEstimator.estimate(state, currentTime, pressure);
                }
            }

    }; // class Ladybug

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
