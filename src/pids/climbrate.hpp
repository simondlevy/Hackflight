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
            static constexpr float KI = 15;
            static constexpr float ILIMIT = 5000;

            float _integral;

        public:

            float run(const float dt, float throttle, const float dz)
            {
                const auto error = throttle - dz;

                _integral = Utils::fconstrain(_integral + dt * error, ILIMIT);

                return  KP * error + KI * _integral;
            }
    };

}
