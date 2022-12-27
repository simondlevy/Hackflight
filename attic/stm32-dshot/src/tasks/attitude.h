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

#include "datatypes.h"
#include "task.h"

class AttitudeTask : public Task {

    public:

        AttitudeTask()
            : Task(100) // Hz
        {
        }

        virtual void fun(
                HackflightCore::data_t * core,
                Task::data_t * data,
                uint32_t time) override
        {
            (void)data;
            (void)time;
            
            // Simulates rocking in the X (phi) axis

            static float phi;
            static int8_t dir = +1;

            phi += .01 * dir;

            if (phi >= 1.0) {
                dir = -1;
            }

            if (phi <= -1.0) {
                dir = +1;
            }


            vehicle_state_t * vstate = &core->vstate;
            vstate->phi = phi;
            vstate->theta = 0.1;
            vstate->psi = 0.1;
        }
};
