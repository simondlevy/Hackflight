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

    /**
        Input is angular rate demand (deg/sec) and actual angular rate from
        gyro; ouputput is arbitrary units scaled for motors
     */
     class YawRatePid {

        private:

            static constexpr float I_LIMIT = 25.0;     

            static constexpr float SCALE = 1e-1;

            static constexpr float KP = SCALE * 0.003;           
            static constexpr float KI = SCALE * 0.0005;          
            static constexpr float KD = SCALE * 0.0000015;       

        public:

            void run(
                    const float dt, 
                    const bool reset,
                    const state_t & state,
                    demands_t & demands) 
            {
                const auto error = demands.yaw - state.dpsi;

                _integral = reset ? 0 :
                    Utils::fconstrain(_integral + error * dt, I_LIMIT);

                const auto derivative = (error - _error) / dt;

                demands.yaw = KP * error + KI * _integral - KD * derivative; 

                _error = error;
              }    

        private:

            float _integral;

            float _error;

    };
}
