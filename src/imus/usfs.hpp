/*
   Support for USFS IMU

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
#include <USFS_Master.h>
#include "imu.hpp"

namespace hf {

    class USFS : public IMU {

        private:

            // Tunable USFS parameters
            static const uint8_t  MAG_RATE       = 100;  // Hz
            static const uint16_t ACCEL_RATE     = 330;  // Hz
            static const uint16_t GYRO_RATE      = 330;  // Hz
            static const uint8_t  BARO_RATE      = 50;   // Hz
            static const uint8_t  Q_RATE_DIVISOR = 5;    // 1/5 gyro rate

            USFS_Master _sentral = USFS_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

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

            // Handle upside-down mounting
            virtual void adjustGyrometer(float & gx, float & gy, float & gz) { (void)gx; (void)gy; (void)gz;  }
            virtual void adjustQuaternion(float & qw, float & qx, float & qy, float & qz) { (void)qw; (void)qx; (void)qy; (void)qz;  }

        public:

            virtual bool getGyrometer(float & gx, float & gy, float & gz) override
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

                    adjustGyrometer(gx, gy, gz);

                    return true;
                }

                return false;
            }

            virtual bool getQuaternion(float & qw, float & qx, float & qy, float & qz, float time) override
            {
                (void)time;

                if (_sentral.gotQuaternion()) {

                    _sentral.readQuaternion(qw, qx, qy, qz);

                    adjustQuaternion(qw, qx, qy, qz);

                    return true;
                }

                return false;
            }

            virtual void begin(void) override
            {
                // Start the USFS in master mode, no interrupt
                if (!_sentral.begin()) {
                    while (true) {
                        Serial.println(_sentral.getErrorString());
                    }
                }
            }

    }; // class USFS

} // namespace hf
