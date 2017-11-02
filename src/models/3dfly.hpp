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

                // Level (accelerometer)
                levelP         = 0.20f;

                // Rate (gyro): P must be positive
                ratePitchRollP = 0.225f;
                ratePitchRollI = 0.001875f;
                ratePitchRollD = 0.375f;

                // Yaw: P must be positive
                yawP           = 1.0625f;
                yawI           = 0.005625f;

                // Trim for a particular vehicle: roll, pitch, yaw
                softwareTrimRoll  = 0;
                softwareTrimPitch = 0;
                softwareTrimYaw   = 0;

                // Altitude 
                altP = 0.5f;
                altI = 0.02f; 
                altD = 1.7f;
            }
    };

} // namespace
