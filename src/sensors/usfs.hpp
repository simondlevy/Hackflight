/*
   Support for USFS IMU

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

#include <math.h>
#include <Wire.h>
#include <USFS_Master.h>
#include <RFT_sensor.hpp>

namespace hf {

    /*
       The most common aeronautical convention defines roll as acting about
       the longitudinal axis, positive with the starboard (right) wing
       down. Yaw is about the vertical body axis, positive with the nose to
       starboard. Pitch is about an axis perpendicular to the longitudinal
       plane of symmetry, positive nose up.

        https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft)

        https://emissarydrones.com/what-is-roll-pitch-and-yaw
    */

    class USFS {

        friend class Hackflight;
        friend class UsfsQuaternion;
        friend class UsfsGyrometer;

        private:

        // Tunable USFS parameters
        static const uint8_t  MAG_RATE       = 100;  // Hz
        static const uint16_t ACCEL_RATE     = 330;  // Hz
        static const uint16_t GYRO_RATE      = 330;  // Hz
        static const uint8_t  BARO_RATE      = 50;   // Hz
        static const uint8_t  Q_RATE_DIVISOR = 5;    // 1/5 gyro rate

        bool _begun = false;

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

        bool getQuaternion(float & qw, float & qx, float & qy, float & qz, float time)
        {
            (void)time;

            if (_sentral.gotQuaternion()) {

                readSentralQuaternion(qw, qx, qy, qz);

                return true;
            }

            return false;
        }

        void begin(void)
        {
            if (!_begun) {

                // Start the USFS in master mode, no interrupt
                if (!_sentral.begin()) {
                    while (true) {
                        Serial.println(_sentral.getErrorString());
                        delay(100);
                    }
                }
                _begun = true;
            }
        }

    }; // class USFS

    // Singleton
    static USFS _usfs;

    class UsfsGyrometer : public rft::Sensor {

        friend class Hackflight;

        private:

        float _x = 0;
        float _y = 0;
        float _z = 0;

        USFS * _imu = &_usfs;

        protected:

        virtual void begin(void) override
        {
            _imu->begin();
        }


        virtual void modifyState(rft::State * state, float time) override
        {
            (void)time;

            State * hfstate = (State *)state;

            // NB: We negate gyro Y, Z to simplify PID controller
            hfstate->x[State::DPHI] = _x;
            hfstate->x[State::DTHETA] = _y;
            hfstate->x[State::DPSI] = _z;
        }

        virtual bool ready(float time) override
        {
            (void)time;

            bool result = _imu->getGyrometer(_x, _y, _z);

            return result;
        }

        public:

        UsfsGyrometer(void)
        {
            _x = 0;
            _y = 0;
            _z = 0;
        }

    };  // class Gyrometer

    class UsfsQuaternion : public rft::Sensor {

        friend class Hackflight;

        private:

        float _w = 0;
        float _x = 0;
        float _y = 0;
        float _z = 0;

        USFS * _imu = &_usfs;

        protected:

        virtual void begin(void) override
        {
            _imu->begin();
        }

        virtual void modifyState(rft::State * state, float time) override
        {
            (void)time;

            State * hfstate = (State *)state;

            float qw = _w, qx = _x, qy = _y, qz = _z;

            computeEulerAngles(qw, qx, qy, qz, 
                    hfstate->x[State::PHI], hfstate->x[State::THETA], hfstate->x[State::PSI]);

            // Adjust rotation so that nose-up is positive
            hfstate->x[State::THETA] = -hfstate->x[State::THETA];

            // Convert heading from [-pi,+pi] to [0,2*pi]
            if (hfstate->x[State::PSI] < 0) {
                hfstate->x[State::PSI] += 2*M_PI;
            }
        }

        virtual bool ready(float time) override
        {
            return _imu->getQuaternion(_w, _x, _y, _z, time);
        }

        public:

        UsfsQuaternion(void)
        {
            _w = 0;
            _x = 0;
            _y = 0;
            _z = 0;
        }

        // We make this public so we can use it in different sketches
        static void computeEulerAngles(float qw, float qx, float qy, float qz,
                float & ex, float & ey, float & ez)
        {
            ex = atan2(2.0f*(qw*qx+qy*qz), qw*qw-qx*qx-qy*qy+qz*qz);
            ey = asin(2.0f*(qx*qz-qw*qy));
            ez = atan2(2.0f*(qx*qy+qw*qz), qw*qw+qx*qx-qy*qy-qz*qz);
        }

    };  // class Quaternion

} // namespace hf
