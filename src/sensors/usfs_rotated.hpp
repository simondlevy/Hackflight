/*
   Support for USFS IMU rotated 90 clockwise

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>
#include <USFS_Master.h>
#include <sensor.hpp>

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
                ex = -asin(2.0f*(qx*qz-qw*qy));
                ey = -atan2(2.0f*(qw*qx+qy*qz), qw*qw-qx*qx-qy*qy+qz*qz);
                ez = atan2(2.0f*(qx*qy+qw*qz), qw*qw+qx*qx-qy*qy-qz*qz);
            }

        protected:

            virtual void begin(void) override 
            {
                _usfs.begin();
            }

            virtual void modifyState(State * state, float time) override
            {
                (void)time;

                float qw = 0;
                float qx = 0;
                float qy = 0;
                float qz = 0;

                _usfs.sentral.readQuaternion(qw, qx, qy, qz);

                computeEulerAngles(qw, qx, qy, qz,
                        state->x[State::STATE_PHI],
                        state->x[State::STATE_THETA],
                        state->x[State::STATE_PSI]);

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (state->x[State::STATE_PSI] < 0) {
                    state->x[State::STATE_PSI] += 2*M_PI;
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

            virtual void modifyState(State * state, float time) override
            {
                (void)time;

                float gx = 0;
                float gy = 0;
                float gz = 0;

                // Returns degrees / sec
                _usfs.sentral.readGyrometer(gx, gy, gz);

                // Convert degrees / sec to radians / sec
                state->x[State::STATE_DPHI] = radians(gy);
                state->x[State::STATE_DTHETA] = -radians(gx);
                state->x[State::STATE_DPSI] = radians(gz);
            }

            virtual bool ready(float time) override
            {
                (void)time;

                _usfs.checkEventStatus();

                return _usfs.sentral.gotGyrometer();
            }

    }; // class UsfsGyro

} // namespace hf
