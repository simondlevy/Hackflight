/*
  Position-hold PID controller

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

    /**
        Input is meter/second demands and actual velocities; output is roll/pitch
        angles
     */
    class PositionPid {

        private:

            static constexpr float KP = 10;

        public:

            static void run(
                    const float rollDemand, const float pitchDemand,
                    const float dx, const float dy,
                    float & newRollDemand, float & newPitchDemand)
            {
                newRollDemand = KP * (rollDemand - dy);
                newPitchDemand = KP * (pitchDemand - dx);
            }

    };

}
