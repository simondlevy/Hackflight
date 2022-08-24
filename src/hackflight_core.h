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

#include "datatypes.h"
#include "mixer.h"
#include "pids/angle.h"
#include "pids/althold.h"

class HackflightCore {

    private:

        static constexpr float PID_MIXER_SCALING = 1000;
        static const uint16_t  PIDSUM_LIMIT_YAW  = 400;
        static const uint16_t  PIDSUM_LIMIT      = 500;

        static float constrain_demand(float demand, float limit, float scaling)
        {
            return constrain_f(demand, -limit, +limit) / scaling;
        }

        static void constrain_demands(demands_t * demands)
        {
            demands->roll  = constrain_demand(
                    demands->roll, PIDSUM_LIMIT, PID_MIXER_SCALING);

            demands->pitch =
                constrain_demand(
                        demands->pitch, PIDSUM_LIMIT, PID_MIXER_SCALING);

            // Negate yaw to make it agree with PID
            demands->yaw   =
                -constrain_demand(
                        demands->yaw, PIDSUM_LIMIT_YAW, PID_MIXER_SCALING);
        }

    public:

        static void step(
                demands_t * demands,
                vehicle_state_t * vstate,
                AnglePidController * anglePid,
                bool pidReset,
                uint32_t usec,
                Mixer * mixer,
                float motorvals[],
                AltHoldPidController * altHoldPid=NULL)
        {
            // Run PID controllers to get new demands
            anglePid->update(usec, demands, vstate, pidReset);

            if (altHoldPid != NULL) {
                altHoldPid->update(usec, demands, vstate, pidReset);
            }

            // Constrain demands
            constrain_demands(demands);

            // Run the mixer to get motors from demands
            mixer->run(demands, motorvals);
        }

}; // class HackflightCore


