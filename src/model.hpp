/*
   model.hpp :  PID values and other tuning for a specific model

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>

namespace hf {

class Model {

    public:

        // Level (accelerometer)
        float levelP = 0;

        // Rate (gyro): P must be positive
        float ratePitchRollP = 0;
        float ratePitchRollI = 0;
        float ratePitchRollD = 0;

        // Yaw: P must be positive
        float yawP = 0;
        float yawI = 0;

        // Trim for a particular vehicle
        float softwareTrimRoll  = 0;
        float softwareTrimPitch = 0;
        float softwareTrimYaw   = 0;

        // Altitude 
        float altP = 0;
        float altI = 0; 
        float altD = 0;
};

} // namespace hf
