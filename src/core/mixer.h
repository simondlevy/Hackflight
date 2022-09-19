/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "constrain.h"
#include "demands.h"
#include "motors.h"
#include "pid.h"
#include "vstate.h"

#include <vector>
using namespace std;

class Mixer {

    private:

        typedef Motors (*mixerFun_t)(const Demands & demands);

        uint8_t m_motorCount;

        mixerFun_t m_fun;

    public:

        Mixer(const uint8_t motorCount, const mixerFun_t fun)
        {
            m_motorCount = motorCount;
            m_fun = fun;
        }

        uint8_t getMotorCount(void)
        {
            return m_motorCount;
        }

        auto step(
                const Demands & stickDemands,
                const VehicleState & state,
                vector<PidController *> * pidControllers,
                const bool pidReset,
                const uint32_t usec) -> Motors
        {
            // Star with stick demands
            Demands demands(stickDemands);

            for (auto p: *pidControllers) {
                demands = p->update(usec, demands, state, pidReset);
            }

            // Run the mixer to get motors from demands
            return m_fun(demands);
        }
};
