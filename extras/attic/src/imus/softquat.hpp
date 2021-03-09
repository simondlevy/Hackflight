/*
   Abstract class for IMUs that need to compute the quaternion on the MCU

   Copyright (c) 2020 Simon D. Levy

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

#include "filters.hpp"
#include "imu.hpp"

#include <math.h>

namespace hf {

    class SoftwareQuaternionIMU : public IMU {

        private:

            // Global constants for 6 DoF quaternion filter
            const float GYRO_MEAS_ERROR_DEG = 20.f;
            const float GYRO_MEAS_DRIFT_DEG =  0.f;

            // Update quaternion after this number of gyro updates
            const uint8_t QUATERNION_DIVISOR = 5;

            // Supports computing quaternion after a certain number of IMU readings
            uint8_t _quatCycleCount = 0;

            // Params passed to Madgwick quaternion constructor
            const float _beta = sqrtf(3.0f / 4.0f) * Filter::deg2rad(GYRO_MEAS_ERROR_DEG);
            const float _zeta = sqrtf(3.0f / 4.0f) * Filter::deg2rad(GYRO_MEAS_DRIFT_DEG);  

            float _ax = 0;
            float _ay = 0;
            float _az = 0;
            float _gx = 0;
            float _gy = 0;
            float _gz = 0;

        protected:

            // Quaternion support: even though MPU9250 has a magnetometer, we keep it simple for now by 
            // using a 6DOF fiter (accel, gyro)
            MadgwickQuaternionFilter6DOF _quaternionFilter = MadgwickQuaternionFilter6DOF(_beta, _zeta);

            virtual bool imuReady(void) = 0;

            virtual void imuReadAccelGyro(float & ax, float & ay, float & az, float & gx, float & gy, float &gz) = 0;

        public:

            bool getGyrometer(float & gx, float & gy, float & gz) override
            {
                // Read acceleromter Gs, gyrometer rad/sec
                if (imuReady()) {

                    imuReadAccelGyro(_ax, _ay, _az, _gx, _gy, _gz);

                    return true;
                }

                return false;
            }

            bool getQuaternion(float & qw, float & qx, float & qy, float & qz, float time) override
            {
                // Update quaternion after some number of IMU readings
                _quatCycleCount = (_quatCycleCount + 1) % QUATERNION_DIVISOR;

                if (_quatCycleCount == 0) {

                    // Set integration time by time elapsed since last filter update
                    static float _time;
                    float deltat = time - _time;
                    _time = time;

                    // Run the quaternion on the IMU values acquired in imuReadAccelGyro()                   
                    _quaternionFilter.update(_ax, _ay, _az, _gx, _gy, _gz, deltat); 

                    // Copy the quaternion back out
                    qw = _quaternionFilter.q1;
                    qx = _quaternionFilter.q2;
                    qy = _quaternionFilter.q3;
                    qz = _quaternionFilter.q4;

                    return true;
                }

                return false;
            }

    }; // class SoftwareQuaternionIMU

} // namespace hf
