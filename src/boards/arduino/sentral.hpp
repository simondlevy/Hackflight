/*
   Support for EM7180 SENtral Sensor Fusion solution

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

            SentralBoard(uint8_t ledPin, bool ledInverted=false) 
                : ArduinoBoard(ledPin, ledInverted)
            {
            }

            bool getGyrometer(float & gx, float & gy, float & gz)
            {
                // Since gyro is updated most frequently, use it to drive SENtral polling
                checkEventStatus();

                if (_sentral.gotGyrometer()) {

                    // Returns degrees / sec
                    _sentral.readGyrometer(gx, gy, gz);

                    // Convert degrees / sec to radians / sec
                    gx = radians(gx);
                    gy = radians(gy);
                    gz = radians(gz);

                    return true;
                }

                return false;
            }

            bool getQuaternion(float & qw, float & qx, float & qy, float & qz)
            {
                if (_sentral.gotQuaternion()) {

                    _sentral.readQuaternion(qw, qx, qy, qz);

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
