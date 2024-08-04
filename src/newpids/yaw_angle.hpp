/*
   Yaw angle PID controller for Hackflight

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

    class YawAngleController {

        /*
           Demand is input as desired angle normalized to [-1,+1] and output
           as degrees per second, both nose-right positive.
         */

        public:

            void run(const state_t & state, const float target,
                    demands_t & demands)
            {
                demands.yaw = KP * (target - state.psi);

            }

        private:

            static constexpr float KP = 6;

    };

}