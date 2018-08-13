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

namespace hf {

    class SoftwareQuaternionBoard : public RealBoard {

        private:

            // Global constants for 6 DoF quaternion filter
            const float GYRO_MEAS_ERROR = M_PI * (40.0f / 180.0f);
            const float GYRO_MEAS_DRIFT = M_PI * (0.0f  / 180.0f);
            const float BETA = sqrtf(3.0f / 4.0f) * GYRO_MEAS_ERROR; 
            const float ZETA = sqrt(3.0f / 4.0f) * GYRO_MEAS_DRIFT;  

            // Update quaternion after this number of gyro updates
            const uint8_t QUATERNION_DIVISOR = 5;

            // Instance variables -----------------------------------------------------------------------------------

            // Quaternion support: even though MPU9250 has a magnetometer, we keep it simple for now by 
            // using a 6DOF fiter (accel, gyro)
            MadgwickQuaternionFilter6DOF _quaternionFilter = MadgwickQuaternionFilter6DOF(BETA, ZETA);
            uint8_t _quatCycleCount = 0;

        protected:

            float _ax = 0;
            float _ay = 0;
            float _az = 0;
            float _gx = 0;
            float _gy = 0;
            float _gz = 0;

            bool getQuaternion(float quat[4])
            {
                // Update quaternion after some number of IMU readings
                _quatCycleCount = (_quatCycleCount + 1) % QUATERNION_DIVISOR;

                if (_quatCycleCount == 0) {

                    // Set integration time by time elapsed since last filter update
                    static float _time;
                    float time = getTime();
                    float deltat = time - _time;
                    _time = time;

                    // Run the quaternion on the IMU values acquired in getGyrometer()
                    _quaternionFilter.update(-_ax, _ay, _az, _gx, -_gy, -_gz, deltat); // Butterfly
                    //_quaternionFilter.update(_ay, _ax,  _az, -_gy, -_gx, -_gz, deltat);  // AlienflightF3

                    // Copy the quaternion back out
                    quat[0] = _quaternionFilter.q1;
                    quat[1] = _quaternionFilter.q2;
                    quat[2] = _quaternionFilter.q3;
                    quat[3] = _quaternionFilter.q4;

                    return true;
                }

                return false;
            }

    }; // class SoftwareQuaternionBoard

} // namespace hf
