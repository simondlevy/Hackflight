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

                // Level (Euler angles): set to zero for rate mode
                levelP = 0.10f;

                // Rate (gyro): P must be positive
                gyroCyclicP = .00001f;// 0.125f;
                gyroCyclicI = 0;// 0.05f;
                gyroCyclicD = 0;// 0.01f;

                // Yaw: P must be positive
                gyroYawP = 0.0001f;// 0.1f;
                gyroYawI = 0;// 0.05f;

                // Altitude 
                altP = 0.04f;
                altI = 0.50f;
                altD = 6.00f;
             }
    };

} // namespace hf
