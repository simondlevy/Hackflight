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

#include "msp.h"
#include "task.h"

class MspTask : public Task {

    public:

        MspTask() : Task(100) { } // Hz

        virtual void fun(
                HackflightCore::data_t * core,
                Task::data_t * data,
                uint32_t usec) override
        {
            (void)usec;

            mspUpdate(
                    &core->vstate,
                    &data->rxAxes,
                    armingIsArmed(&data->arming),
                    data->motorDevice,
                    data->mspMotors);
        }
};
