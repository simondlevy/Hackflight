/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "constrain.h"
#include "demands.h"
#include "mixer.h"
#include "pid.h"
#include "state.h"

class HackflightCore {

    private:

        static constexpr float PID_MIXER_SCALING = 1000;
        static const uint16_t  PIDSUM_LIMIT_YAW  = 400;
        static const uint16_t  PIDSUM_LIMIT      = 500;

        static float constrain_demand(const float demand, const float limit)
        {
            return constrain(demand, -limit, +limit) / PID_MIXER_SCALING;
        }

        static void constrain_demands(Demands * demands)
        {
            demands->roll  = constrain_demand(demands->roll, PIDSUM_LIMIT);

            demands->pitch = constrain_demand(demands->pitch, PIDSUM_LIMIT);

            // Negate yaw to make it agree with PID
            demands->yaw   = -constrain_demand(demands->yaw, PIDSUM_LIMIT_YAW);
        }

    public:

        static auto step(
                const Demands & stickDemands,
                const State & state,
                PidController * pidControllers[],
                const uint8_t pidCount,
                const bool pidReset,
                const uint32_t usec,
                Mixer mixer) -> Motors
        {
            // Star with stick demands
            Demands demands(stickDemands);

            // Run PID controllers to get new demands
            for (auto k=0; k<pidCount; ++k) {
                pidControllers[k]->update(usec, &demands, state, pidReset);
            }

            // Constrain demands
            constrain_demands(&demands);

            // Run the mixer to get motors from demands
            return mixer.run(demands);
        }

};  // class HackflightCore
