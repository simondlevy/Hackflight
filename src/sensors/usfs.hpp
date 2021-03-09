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
#include "sensor.hpp"

namespace hf {

    // Singleton class
    class _USFS {

        friend class UsfsQuat;
        friend class UsfsGyro;

        private:

            // Tunable USFS parameters
            static const uint8_t  MAG_RATE       = 100;  // Hz
            static const uint16_t ACCEL_RATE     = 330;  // Hz
            static const uint16_t GYRO_RATE      = 330;  // Hz
            static const uint8_t  BARO_RATE      = 50;   // Hz
            static const uint8_t  Q_RATE_DIVISOR = 5;    // 1/5 gyro rate

            bool _begun = false;

        protected:

            void checkEventStatus(void)
            {
                sentral.checkEventStatus();

                if (sentral.gotError()) {
                    while (true) {
                        Serial.print("ERROR: ");
                        Serial.println(sentral.getErrorString());
                    }
                }
            }

            USFS_Master sentral = USFS_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

            void begin(void)
            {
                if (_begun) return;

                // Start the USFS in master mode, no interrupt
                if (!sentral.begin()) {
                    while (true) {
                        Serial.println(sentral.getErrorString());
                        delay(100);
                    }
                }
 
                _begun = true;
            }

     }; // class _USFS

    static _USFS  _usfs;

    class UsfsQuat : public Sensor {

        private:

            static void computeEulerAngles(float qw, float qx, float qy, float qz,
                                           float & ex, float & ey, float & ez)
            {
                ex = atan2(2.0f*(qw*qx+qy*qz), qw*qw-qx*qx-qy*qy+qz*qz);
                ey = asin(2.0f*(qx*qz-qw*qy));
                ez = atan2(2.0f*(qx*qy+qw*qz), qw*qw+qx*qx-qy*qy-qz*qz);
            }

        protected:

            virtual void begin(void) override 
            {
                _usfs.begin();
            }

            virtual void modifyState(state_t & state, float time) override
            {
                (void)time;

                float qw = 0;
                float qx = 0;
                float qy = 0;
                float qz = 0;

                _usfs.sentral.readQuaternion(qw, qx, qy, qz);

                computeEulerAngles(qw, qx, qy, qz, state.x[STATE_PHI], state.x[STATE_THETA], state.x[STATE_PSI]);

                // Adjust rotation so that nose-up is positive
                state.x[STATE_THETA] = -state.x[STATE_THETA];

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (state.x[STATE_PSI] < 0) {
                    state.x[STATE_PSI] += 2*M_PI;
                }
            }

            virtual bool ready(float time) override
            {
                (void)time;

                _usfs.checkEventStatus();

                return _usfs.sentral.gotQuaternion();
            }

    }; // class UsfsQuat


    class UsfsGyro : public Sensor {

        protected:

            virtual void begin(void) override 
            {
                _usfs.begin();
            }

            virtual void modifyState(state_t & state, float time) override
            {
                (void)time;

                float gx = 0;
                float gy = 0;
                float gz = 0;

                // Returns degrees / sec
                _usfs.sentral.readGyrometer(gx, gy, gz);

                // Convert degrees / sec to radians / sec
                state.x[STATE_DPHI] = radians(gx);
                state.x[STATE_DTHETA] = radians(gy);
                state.x[STATE_DPSI] = radians(gz);
            }

            virtual bool ready(float time) override
            {
                (void)time;

                _usfs.checkEventStatus();

                return _usfs.sentral.gotGyrometer();
            }

    }; // class UsfsGyro

} // namespace hf
