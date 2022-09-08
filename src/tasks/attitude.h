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

        Arming * m_arming;
        Imu *    m_imu;

        float m_maxArmingAngle = Math::deg2rad(MAX_ARMING_ANGLE);

    protected:

        AttitudeTask()
            : Task(100) // Hz
        {
        }

        void begin(Imu * imu, Arming * arming)
        {
            m_imu = imu;
            m_arming = arming;
        }

        virtual void fun(Task::data_t * data, uint32_t time) override
        {
            m_imu->getEulerAngles(m_arming->isArmed(), time, &data->vstate);

            auto imuIsLevel =
                fabsf(data->vstate.phi) < m_maxArmingAngle &&
                fabsf(data->vstate.theta) < m_maxArmingAngle;

            m_arming->updateImuStatus(imuIsLevel, m_imu->gyroIsCalibrating()); 
        }
};
