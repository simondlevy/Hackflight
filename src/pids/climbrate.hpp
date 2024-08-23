/*
  Climb-rate PID controller

  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

namespace hf {

    class ClimbRatePid {

        private:

            static constexpr float KP = 25;

            static constexpr float THRUST_TAKEOFF = 56;

            static constexpr float THRUST_BASE = 55.385;

        public:

            static float run(
                    const bool hitTakeoffButton, 
                    const bool completedTakeoff,
                    const float throttle,
                    const float dz)
            {
                return
                    completedTakeoff ? 
                    THRUST_BASE + KP * (throttle - dz) :
                    hitTakeoffButton ?
                    THRUST_TAKEOFF :
                    0;
            }

    };

}
