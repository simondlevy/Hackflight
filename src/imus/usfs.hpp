/*
   Support for USFS IMU

   Copyright (c) 2018 Simon D. Levy

   MIT License
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

            USFS_Master _sentral = USFS_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

            virtual void readSentralQuaternion(float & qw, float & qx, float & qy, float & qz)
            {
                _sentral.readQuaternion(qw, qx, qy, qz);
            }

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

                    return true;
                }

                return false;
            }

            virtual bool getQuaternion(float & qw, float & qx, float & qy, float & qz, float time) override
            {
                (void)time;

                if (_sentral.gotQuaternion()) {

                    readSentralQuaternion(qw, qx, qy, qz);

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
