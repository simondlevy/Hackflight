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
#include "debug.h"
#include "pids/angle.h"

class HackflightCore {

    private:

        static constexpr float PID_MIXER_SCALING = 1000;
        static const uint16_t  PIDSUM_LIMIT_YAW  = 400;
        static const uint16_t  PIDSUM_LIMIT      = 500;

        static float constrain_demand(float demand, float limit, float scaling)
        {
            return constrain_f(demand, -limit, +limit) / scaling;
        }

    public:

        typedef struct {

            anglePid_t       anglePid;
            demands_t        demands;
            mixer_t          mixer;
            pid_controller_t pidControllers[10];
            uint8_t          pidCount;
            bool             pidReset;
            vehicle_state_t  vstate;

        } data_t;

        static void init(
                data_t * hc,
                anglePidConstants_t * anglePidConstants,
                mixer_t mixer)
        {
            hc->mixer = mixer;

            anglePidInit(&hc->anglePid, anglePidConstants);

            addPidController(hc, anglePidUpdate, &hc->anglePid);
        }

        static void addPidController(
                data_t * hc, pid_fun_t fun, void * data)
        {
            hc->pidControllers[hc->pidCount].fun = fun;
            hc->pidControllers[hc->pidCount].data = data;
            hc->pidCount++;
        }

        static void step(
                data_t * hc,
                uint32_t usec,
                bool failsafe,
                motor_config_t * motorConfig,
                float motorvals[])
        {
            // Run PID controllers to get new demands
            for (uint8_t k=0; k<hc->pidCount; ++k) {
                pid_controller_t pid = hc->pidControllers[k];
                pid.fun(
                        usec,
                        &hc->demands,
                        pid.data,
                        &hc->vstate,
                        hc->pidReset);
            }

            // Constrain the demands, negating yaw to make it agree with PID
            demands_t * demands = &hc->demands;
            demands->roll  = constrain_demand(
                    demands->roll, PIDSUM_LIMIT, PID_MIXER_SCALING);
            demands->pitch =
                constrain_demand(
                        demands->pitch, PIDSUM_LIMIT, PID_MIXER_SCALING);
            demands->yaw   =
                -constrain_demand(
                        demands->yaw, PIDSUM_LIMIT_YAW, PID_MIXER_SCALING);

            // Run the mixer to get motors from demands
            hc->mixer(&hc->demands, failsafe, motorConfig, motorvals);
        }

}; // class HackflightCore


