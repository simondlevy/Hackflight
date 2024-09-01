/*
   Yaw PID controller

   Adapted from https://github.com/nickrehm/dRehmFlight

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

#include <hackflight.hpp>
#include <utils.hpp>

namespace hf {

    class YawPid {

        private:

            static constexpr float I_LIMIT = 25.0;     

            static constexpr float KP = 0.003;           
            static constexpr float KI = 0.0005;          
            static constexpr float KD = 0.0000015;       

        public:

            float run(
                    const float dt, 
                    const bool reset,
                    const float demand, 
                    const float dangle)
            {
                const auto error = demand - dangle;

                _integral = reset ? 0 :
                    hf::Utils::fconstrain(_integral + error * dt, I_LIMIT);

                const auto derivative = (error - _error) / dt;

                const auto output =
                    KP * error + KI * _integral - KD * derivative; 

                _error = error;

                return output;
              }    

        private:

            float _integral;

            float _error;

    };
}
