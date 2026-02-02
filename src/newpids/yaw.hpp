/*
   Yaw-rate PID controller for Hackflight

   Based on  https://github.com/nickrehm/dRehmFlight

   Copyright (C) 2026 Simon D. Levy

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

#include <hackflight.h>
#include <datatypes.h>
#include <num.hpp>

namespace hf {

    class YawPid {

        private:

            static constexpr float I_LIMIT = 25.0;     

            static constexpr float KP = 0.3;           
            static constexpr float KI = 0.05;          
            static constexpr float KD = 0.00015;       

            float _integral;
            float _error;

        public:

            float run(
                    const float dt,
                    const bool airborne,
                    const float target,
                    const float actual)
            {
                const auto error = target - actual;

                const auto integral = airborne ? 
                    Num::fconstrain(_integral + error * dt, I_LIMIT) : 0;

                _integral = integral;

                const auto derivative = (error - _error) / dt; 

                _error = error;

                return .01 * (KP*error + KI*integral + KD*derivative); 
            }
    };

}
