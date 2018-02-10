/*
   imu.hpp: Altitude estimation via accelerometer Z-axis integration

   Adapted from

   https://github.com/multiwii/baseflight/blob/master/src/imu.c

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
   along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>

#include "filter.hpp"

namespace hf {

    class IMU {

        private:

            const float      ACCEL_LPF_CUTOFF  = 5.0f;
            const float      GYRO_SCALE        = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;
            const float      ACCEL_LPF_FACTOR  = 4.f;
            const float      ACCEL_Z_DEADBAND  = 40.f;

            int16_t accADC[3];
            int16_t gyroADC[3];

            float anglerad[2] = { 0.0f, 0.0f };    // absolute angle inclination in radians
            int16_t angle[2] = { 0, 0 };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
            int16_t heading;
            float EstG[3];
            float accSmooth[3];
            uint32_t previousTime;
            float accVelScale;
            float fc_acc;
            int32_t accSumZ;
            uint32_t accTimeSum;        // keep track for integration of acc
            int accSumCount;
            int32_t accZoffset;
            float accZsmooth;
            bool ready;

            // Rotate Estimated vector(s) with small angle approximation, according to the gyro data
            static void rotateV(float *vout, float *delta)
            {
                float v_tmp[3];
                memcpy(v_tmp, vout, 3*sizeof(float));

                // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
                float mat[3][3];
                float cosx, sinx, cosy, siny, cosz, sinz;
                float coszcosx, sinzcosx, coszsinx, sinzsinx;

                cosx = cosf(delta[0]);
                sinx = sinf(delta[0]);
                cosy = cosf(delta[1]);
                siny = sinf(delta[1]);
                cosz = cosf(delta[2]);
                sinz = sinf(delta[2]);

                coszcosx = cosz * cosx;
                sinzcosx = sinz * cosx;
                coszsinx = sinx * cosz;
                sinzsinx = sinx * sinz;

                mat[0][0] = cosz * cosy;
                mat[0][1] = -cosy * sinz;
                mat[0][2] = siny;
                mat[1][0] = sinzcosx + (coszsinx * siny);
                mat[1][1] = coszcosx - (sinzsinx * siny);
                mat[1][2] = -sinx * cosy;
                mat[2][0] = (sinzsinx) - (coszcosx * siny);
                mat[2][1] = (coszsinx) + (sinzcosx * siny);
                mat[2][2] = cosy * cosx;

                vout[0] = v_tmp[0] * mat[0][0] + v_tmp[1] * mat[1][0] + v_tmp[2] * mat[2][0];
                vout[1] = v_tmp[0] * mat[0][1] + v_tmp[1] * mat[1][1] + v_tmp[2] * mat[2][1];
                vout[2] = v_tmp[0] * mat[0][2] + v_tmp[1] * mat[1][2] + v_tmp[2] * mat[2][2];
            }

            void update(uint32_t currentTime)
            {
                uint32_t deltaTime = currentTime - previousTime;
                float scale = deltaTime * GYRO_SCALE;
                previousTime = currentTime;

                // Initialization
                float deltaGyroAngle[3];
                for (uint8_t axis = 0; axis < 3; axis++) {
                    deltaGyroAngle[axis] = gyroADC[axis] * scale;
                    accSmooth[axis] = accSmooth[axis] * (1.0f - (1.0f / ACCEL_LPF_FACTOR)) + accADC[axis] * (1.0f / ACCEL_LPF_FACTOR);
                }

                rotateV(EstG, deltaGyroAngle);

                // Attitude of the estimated vector
                anglerad[0] = atan2f(EstG[1], EstG[2]);
                anglerad[1] = atan2f(-EstG[0], sqrtf(EstG[1] * EstG[1] + EstG[2] * EstG[2]));
                angle[0] = lrintf(anglerad[0] * (1800.0f / M_PI));
                angle[1] = lrintf(anglerad[1] * (1800.0f / M_PI));

                // deltaTime is measured in us ticks
                float dT = (float)deltaTime * 1e-6f;

                // the accel values have to be rotated into the earth frame
                float rpy[3];
                rpy[0] = -(float)anglerad[0];
                rpy[1] = -(float)anglerad[1];
                rpy[2] = -(float)heading * M_PI / 180.0f;;

                float accel_ned[3];
                accel_ned[0] = accSmooth[0];
                accel_ned[1] = accSmooth[1];
                accel_ned[2] = accSmooth[2];

                IMU::rotateV(accel_ned, rpy);

                accZoffset -= accZoffset / 64;
                accZoffset += accel_ned[2];

                accel_ned[2] -= accZoffset / 64;  // compensate for gravitation on z-axis
                accZsmooth = accZsmooth + (dT / (fc_acc + dT)) * (accel_ned[2] - accZsmooth); // low pass filter

                // apply Deadband to reduce integration drift and vibration influence and
                // sum up Values for later integration to get velocity and distance
                accSumZ += Filter::deadband(lrintf(accZsmooth), ACCEL_Z_DEADBAND);

                accTimeSum += deltaTime;
                accSumCount++;
            }


            void reset(void)
            {
                accSumZ = 0;
                accSumCount = 0;
                accTimeSum = 0;
            }

        public:

            void init(uint16_t accel1G)
            {
                accVelScale = 9.80665f / accel1G / 10000.0f;
                fc_acc = 0.5f / (M_PI * ACCEL_LPF_CUTOFF); // calculate RC time constant used in the accZ lpf

                reset();

                ready = false;
            }

            float getVerticalVelocity(void)
            {
                // Integrator - velocity, cm/sec
                float accZ_tmp = (float)accSumZ / (float)accSumCount;

                // Skip startup transient
                float vel_acc = ready ? accZ_tmp * accVelScale * (float)accTimeSum : 0;

                reset();

                ready = true;

                return vel_acc;
            }

            void updateAccel(int16_t _accADC[3], uint32_t currentTime)
            {
                memcpy(accADC, _accADC, 3*sizeof(float));
                update(currentTime);
            }

            void updateGyro(int16_t _gyroADC[3], uint32_t currentTime)
            {
                memcpy(gyroADC, _gyroADC, 3*sizeof(float));
                update(currentTime);
            }

    }; // class IMU

} // namespace hf
