/*
   Yaw angular rate PID controller for Hackflight

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

#include <utils.hpp>
#include <pid.hpp>

namespace hf {

    class YawRateController {

        /*
           Demand is input in degrees per second and output in units appropriate
           for our motors, both nose-right positive.
         */

        public:

            void run(
                    const float kp,
                    const state_t & state, 
                    const float dt, 
                    demands_t & demands)
            {
                demands.yaw =
                    _pid.run_p(kp, dt, demands.yaw, state.dpsi);
            }

        private:

            PID _pid;
    };

}
