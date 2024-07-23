#pragma once

#include <datatypes.h>
#include <utils.hpp>

class MadgwickFilter {

    public:

        void init(void)
        {
            _q0 = 1;
            _q1 = 0;
            _q2 = 0;
            _q3 = 0;
        }

        /**
          * @param dt time constant
          * @param gyro gyro values in deg/sec
          * @param accel accel values in gs
          * @param quat quaternion output
          */
        void getQuat(
                const float dt, 
                const axis3_t & gyro, 
                const axis3_t & accel, 
                quat_t & quat)
        {
            // Convert gyroscope degrees/sec to radians/sec
            const auto gx = Utils::DEG2RAD * gyro.x;
            const auto gy = Utils::DEG2RAD * gyro.y;
            const auto gz = Utils::DEG2RAD * gyro.z;

            // Rate of change of quaternion from gyro
            auto qDot1 = (-_q1 * gx - _q2 * gy - _q3 * gz) / 2;
            auto qDot2 = (_q0 * gx + _q2 * gz - _q3 * gy) / 2;
            auto qDot3 = (_q0 * gy - _q1 * gz + _q3 * gx) / 2;
            auto qDot4 = (_q0 * gz + _q1 * gy - _q2 * gx) / 2;

            // Compute feedback only if accelerometer measurement valid (avoids NaN in
            // accelerometer normalisation)
            if (!((accel.x == 0) && (accel.y == 0) && (accel.z == 0))) {

                auto ax = accel.x;
                auto ay = accel.y;
                auto az = accel.z;

                // Normalize accelerometer measurement
                const auto recipNorm = invSqrt(ax * ax + ay * ay + az * az);

                ax *= recipNorm;
                ay *= recipNorm;
                az *= recipNorm;

                // Gradient-decent algorithm corrective step

                auto s0 = 4*_q0 * _q2*_q2 + 2*_q2*ax + 4*_q0*_q1*_q1 - 2*_q1*ay;

                auto s1 = 4*_q1 * _q3*_q3 - 2*_q3*ax + 4*_q0*_q0 * _q1 - 
                    2*_q0*ay - 4*_q1 + 8*_q1 * _q1*_q1 + 8*_q1 * _q2*_q2 +
                    4*_q1 * az;

                auto s2 = 4 * _q0*_q0 * _q2 + 2*_q0 * ax + 4*_q2 * _q3*_q3 -
                    2*_q3 * ay - 4*_q2 + 8*_q2 * _q1*_q1 + 8*_q2 * _q2*_q2 +
                    4*_q2 * az;

                auto s3 = 4 * _q1*_q1 * _q3 - 2*_q1 * ax + 4 * _q2*_q2 * _q3 -
                    2*_q2 * ay;

                normalize(s0, s1, s2, s3);

                // Apply feedback step
                qDot1 -= B_madgwick * s0;
                qDot2 -= B_madgwick * s1;
                qDot3 -= B_madgwick * s2;
                qDot4 -= B_madgwick * s3;
            }

            // Integrate rate of change of quaternion to yield quaternion
            _q0 += qDot1 * dt;
            _q1 += qDot2 * dt;
            _q2 += qDot3 * dt;
            _q3 += qDot4 * dt;

            // Normalise quaternion
            normalize(_q0, _q1, _q2, _q3);
            
            quat.w = _q0;
            quat.x = _q1;
            quat.y = _q2;
            quat.z = _q3;
        }

    private:

        static void normalize(float & q0, float & q1, float & q2, float & q3) 
        {
            const auto recipNorm =
                invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

            q0 *= recipNorm;
            q1 *= recipNorm;
            q2 *= recipNorm;
            q3 *= recipNorm;            
         }

        static constexpr float B_madgwick = 0.04; 

        float _q0;
        float _q1;
        float _q2;
        float _q3;

        static float invSqrt(float x) 
        {
            return 1 / sqrtf(x); 
        }
};
