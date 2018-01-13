/*
   3dfly.hpp : 3DFly model (PIDs) for Hackflight

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

#include "model.hpp"

namespace hf {

    class ThreeDFly : public Model {

        public:

            ThreeDFly(void) {

                // Level (Euler angles)
                levelP = 0.20f;

                // Rate (gyro): P must be positive
                gyroCyclicP = 0.225f;
                gyroCyclicI = 0.001875f;
                gyroCyclicD = 0.375f;

                // Yaw: P must be positive
                gyroYawP = 1.0625f;
                gyroYawI = 0.005625f;

                // Trim for a particular vehicle: roll, pitch, gyroYaw
                softwareTrimRoll  =   0.f;
                softwareTrimPitch = -.018f;
                softwareTrimYaw   =   0.f;

                // Altitude 
                altP = 0.1f;
                altI = 0;//0.02f; 
                altD = 0;//1.7f;
            }
    };

} // namespace
