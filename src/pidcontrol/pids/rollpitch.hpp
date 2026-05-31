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

#pragma once

#include <hackflight.h>
#include <datatypes.hpp>
#include <num.hpp>

namespace hf {

    class RollPitchPid {

        private:


            static constexpr float KP = 0.002;    
            static constexpr float KI = 0.003;    
            static constexpr float KD = 0.0005;   
            static constexpr float ILIMIT = 25.0;     

        public:

            float output;

            RollPitchPid() = default;

            RollPitchPid(const RollPitchPid & a) 
                : output(a.output), _integral(a._integral) {}

            RollPitchPid(const float output, const float integral)
                : output(output), _integral(integral) {}

            RollPitchPid& operator=(const RollPitchPid&) = default;

            /**
             * Demand is input as angles in degrees and output as arbitrary
             * values to be scaled according to vehicle
             * characteristics.
             */
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
                    Num::fconstrain(p._integral + error * dt, ILIMIT) : 0;

                const auto output = KP * error + KI * integral - KD * dangle; 

                return RollPitchPid(output, integral);
            }

        private:

            float _integral;

    };
}
