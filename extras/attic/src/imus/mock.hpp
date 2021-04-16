/*
   Mock IMU for testing

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

#include "imu.hpp"

namespace hf {

    class MockIMU : public IMU {

        public:

            virtual bool getGyrometer(float & gx, float & gy, float & gz) override
            {
                gx = 0;
                gy = 0;
                gz = 0;

                return true;
            }

            virtual bool getQuaternion(float & qw, float & qx, float & qy, float & qz, float time) override
            {
                (void)time;

                qw = 0;
                qx = 0;
                qy = 0;
                qz = 0;

                return true;
            }

    }; // class MockIMU

} // namespace hf
