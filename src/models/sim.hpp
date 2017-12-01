/*
   sim.h: PID and other model values for simulators

   Copyright (C) Simon D. Levy 2017

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

#include <model.hpp>

namespace hf {

    class SimModel : public Model {

        public:

            SimModel(void) {

                // Level (accelerometer): set to zero for rate mode
                levelP = 0;//0.10f;

                // Rate (gyro): P must be positive
                ratePitchRollP = .00001f;// 0.125f;
                ratePitchRollI = 0;// 0.05f;
                ratePitchRollD = 0;// 0.01f;

                // Yaw: P must be positive
                yawP = 0.0001f;// 0.1f;
                yawI = 0;// 0.05f;

                // Altitude 
                altP = .04f;
                altI = 0.5f;
                altD = 6.0f;
             }
    };

} // namespace hf
