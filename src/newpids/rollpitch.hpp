/*
   Roll/pitch angle PID controller for Hackflight

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

    class RollPitchPid {

        private:

            static constexpr float I_LIMIT = 25.0;     

            static constexpr float KP = 0.2;    
            static constexpr float KI = 0.3;    
            static constexpr float KD = 0.05;   

        public:

            float output;

            RollPitchPid() = default;

            RollPitchPid(const RollPitchPid & a) 
                : output(a.output), _integral(a._integral) {}

            RollPitchPid(const float output, const float integral)
                : output(output), _integral(integral) {}

            RollPitchPid& operator=(const RollPitchPid&) = default;

            static auto run(
                    const RollPitchPid & p,
                    const float dt,
                    const bool airborne,
                    const float target,
                    const float angle,
                    const float dangle) -> RollPitchPid
            {
                const auto error = target - angle;

                const auto integral = airborne ? 
                    Num::fconstrain(p._integral + error * dt, I_LIMIT) : 0;

                const auto output = 0.01 * (KP * error + KI * integral - KD * dangle); 

                return RollPitchPid(output, integral);
            }

            float run(
                    const float dt,
                    const bool airborne,
                    const float target,
                    const float angle,
                    const float dangle)
            {
                const auto error = target - angle;

                const auto integral = airborne ? 
                    Num::fconstrain(_integral + error * dt, I_LIMIT) : 0;

                _integral = integral;

                return 0.01 * (KP * error + KI * integral - KD * dangle); 
            }

        private:

            float _integral;

    };
}
