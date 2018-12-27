/*
   sentral.hpp : support for EM7180 SENtral Sensor Fusion solution

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
#include <EM7180_Master.h>
#include "arduino.hpp"

namespace hf {

    class SentralBoard : public ArduinoBoard {

        private:

            // Tunable EM7180 parameters
            static const uint8_t  MAG_RATE       = 100;  // Hz
            static const uint16_t ACCEL_RATE     = 330;  // Hz
            static const uint16_t GYRO_RATE      = 330;  // Hz
            static const uint8_t  BARO_RATE      = 50;   // Hz
            static const uint8_t  Q_RATE_DIVISOR = 5;    // 1/5 gyro rate

            EM7180_Master _sentral = EM7180_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

        protected:

            SentralBoard(uint8_t ledPin, bool ledInverted=false) : ArduinoBoard(ledPin, ledInverted)
            {
            }

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

            bool getGyrometer(float gyro[3])
            {
                // Since gyro is updated most frequently, use it to drive SENtral polling
                checkEventStatus();

                if (_sentral.gotGyrometer()) {

                    float gx, gy, gz;

                    // Returns degrees / sec
                    _sentral.readGyrometer(gx, gy, gz);

                    // Convert degrees / sec to radians / sec
                    gyro[0] = radians(gx);
                    gyro[1] = radians(gy);
                    gyro[2] = radians(gz);

                    return true;
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

            void begin(void)
            {
                // Start the EM7180 in master mode, no interrupt
                if (!_sentral.begin()) {
                    while (true) {
                        Serial.println(_sentral.getErrorString());
                    }
                }
            }

    }; // class SentralBoard

} // namespace hf
