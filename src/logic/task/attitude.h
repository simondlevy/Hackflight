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

#include "logic/imu.h"
#include "logic/task.h"

class AttitudeTask : public Task {

    private:

        Imu *          m_imu;
        VehicleState * m_vstate;

    public:

        AttitudeTask(VehicleState & vstate)
            : Task(ATTITUDE, 100) // Hz
        {
            m_vstate = &vstate;
        }

        void begin(Imu * imu)
        {
            m_imu = imu;
        }

        virtual void run(const uint32_t usec) override
        {
            const auto angles = m_imu->getEulerAngles(usec);

            m_vstate->phi   = angles.x;
            m_vstate->theta = angles.y;
            m_vstate->psi   = angles.z;
        }

}; // class AttitudeTask
