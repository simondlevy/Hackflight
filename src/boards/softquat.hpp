/*
   sofquat.hpp : Abstract class for boards that need to compute the quaternion on the MCU

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

#include "filters.hpp"
#include "realboard.hpp"

#include <math.h>

namespace hf {

    class SoftwareQuaternionBoard {

        private:

            // Global constants for 6 DoF quaternion filter
            const float GYRO_MEAS_ERROR_DEG = 40.f;
            const float GYRO_MEAS_DRIFT_DEG =  0.f;

            // Update quaternion after this number of gyro updates
            const uint8_t QUATERNION_DIVISOR = 5;

            // Supports computing quaternion after a certain number of IMU readings
            uint8_t _quatCycleCount = 0;

            // Params passed to Madgwick quaternion constructor
            const float _beta = sqrtf(3.0f / 4.0f) * Filter::deg2rad(GYRO_MEAS_ERROR_DEG);
            const float _zeta = sqrtf(3.0f / 4.0f) * Filter::deg2rad(GYRO_MEAS_DRIFT_DEG);  

        protected:

            float _ax = 0;
            float _ay = 0;
            float _az = 0;
            float _gx = 0;
            float _gy = 0;
            float _gz = 0;

            // For debugging
            int16_t _axRaw = 0;
            int16_t _ayRaw = 0;
            int16_t _azRaw = 0;
            int16_t _gxRaw = 0;
            int16_t _gyRaw = 0;
            int16_t _gzRaw = 0;
            int16_t _mxRaw = 0;
            int16_t _myRaw = 0;
            int16_t _mzRaw = 0;

            // Quaternion support: even though MPU9250 has a magnetometer, we keep it simple for now by 
            // using a 6DOF fiter (accel, gyro)
            MadgwickQuaternionFilter6DOF _quaternionFilter = MadgwickQuaternionFilter6DOF(_beta, _zeta);

            virtual bool imuReady(void) = 0;

            virtual void imuReadAccelGyro(void) = 0;

        public:

            bool getGyrometer(float & gx, float & gy, float & gz)
            {
                // Read acceleromter Gs, gyrometer degrees/sec
                if (imuReady()) {

                    imuReadAccelGyro();

                    // Convert gyrometer values from degrees/sec to radians/sec
                    _gx = Filter::deg2rad(_gx);
                    _gy = Filter::deg2rad(_gy);
                    _gz = Filter::deg2rad(_gz);

                    // Round to two decimal places
                    gx = Filter::round2(_gx);
                    gy = Filter::round2(_gy);
                    gz = Filter::round2(_gz);

                    return true;
                }

                return false;
            }

            bool getQuaternion(float & qw, float & qx, float & qy, float & qz, float time)
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

            virtual void getRawImu(
                    int16_t & ax, int16_t & ay, int16_t & az, 
                    int16_t & gx, int16_t & gy, int16_t & gz,
                    int16_t & mx, int16_t & my, int16_t & mz)
            {
                ax = _axRaw;
                ay = _ayRaw;
                az = _azRaw;
                gx = _gxRaw;
                gy = _gyRaw;
                gz = _gzRaw;
                mx = _mxRaw;
                my = _myRaw;
                mz = _mzRaw;
            }

    }; // class SoftwareQuaternionBoard

} // namespace hf
