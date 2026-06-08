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

#pragma once

#include <hackflight.h>
#include <datatypes.hpp>
#include <num.hpp>

namespace hf {

    class YawPid {

        private:

            static constexpr float KP = 0.003;           
            static constexpr float KI = 0.001;          
            static constexpr float KD = 0.0000015;       
            static constexpr float ILIMIT = 25.0;     

        public:

            float output;

            YawPid() = default;

            YawPid(const YawPid & a) 
                : output(a.output), integral_(a.integral_), error_(a.error_) {}

            YawPid(const float output, const float error, const float integral)
                : output(output), integral_(integral), error_(error) {}

            YawPid& operator=(const YawPid&) = default;

            /**
             * Demand is input as target in degrees per second and output as
             * arbitrary value to be scaled according to vehicle
             * characteristics.
             */
             static auto run(
                    const YawPid & p,
                    const float dt,
                    const bool airborne,
                    const float target,
                    const float actual) -> YawPid
            {
                const auto error = target - actual;

                const auto integral = airborne ? 
                    Num::ConstrainFloat(p.integral_ + error * dt, ILIMIT) : 0;

                const auto derivative = dt > 0 ? (error - p.error_) / dt : 0; 

                const auto output = KP*error + KI*integral + KD*derivative; 

                return YawPid(output, integral, error);
            }

        private:

            float integral_;
            float error_;

    };

}
