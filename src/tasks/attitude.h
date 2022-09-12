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

#include "imu.h"
#include "task.h"

class AttitudeTask : public Task {

    friend class Hackflight;

    private:

        static constexpr float MAX_ARMING_ANGLE = 25;

        Arming *       m_arming;
        Imu *          m_imu;
        VehicleState * m_vstate;

        float m_maxArmingAngle = Math::deg2rad(MAX_ARMING_ANGLE);

    protected:

        AttitudeTask()
            : Task(100) // Hz
        {
        }

        void begin(Imu * imu, Arming * arming, VehicleState * vstate)
        {
            m_imu = imu;
            m_arming = arming;
            m_vstate = vstate;
        }

        virtual void fun(uint32_t time) override
        {
            const auto angles = m_imu->getEulerAngles(time);

            m_vstate->phi   = angles.x;
            m_vstate->theta = angles.y;
            m_vstate->psi   = angles.z;

            const auto imuIsLevel =
                fabsf(m_vstate->phi) < m_maxArmingAngle &&
                fabsf(m_vstate->theta) < m_maxArmingAngle;

            m_arming->updateFromImu(imuIsLevel, m_imu->gyroIsCalibrating()); 
        }
};
