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

        public:

            float run(
                    const float dt,
                    const bool reset,
                    const float demand,
                    const float dangle,
                    float & output)
            {
                const auto error = demand - dangle;

                _integral += error * dt;

                if (reset) {
                    _integral = 0;
                }

                _integral = hf::Utils::fconstrain(
                        _integral, -I_LIMIT, I_LIMIT); 

                const auto derivative = (error - _error_prev) / dt;

                output = KP_YAW * error + KI_YAW * _integral - KD_YAW * derivative; 

                _error_prev = error;

                return output;
            }


        private:

            static constexpr float I_LIMIT = 25.0;     

            static constexpr float KP_YAW = 0.003;           
            static constexpr float KI_YAW = 0.0005;          
            static constexpr float KD_YAW = 0.0000015;       

            static constexpr float THROTTLE_DOWN = 0.06;

            float _integral;
            float _error_prev;
    };
}
