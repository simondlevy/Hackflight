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

#include <string.h>

#include "datatypes.h"
#include "receiver.h"
#include "task.h"

static void task_rx(
        HackflightCore::data_t * core,
        Task::data_t * data,
        uint32_t usec)
{
    bool calibrating = data->gyro.isCalibrating; // || acc.calibrating != 0;
    bool pidItermResetReady = false;
    bool pidItermResetValue = false;

    Receiver::axes_t rxax = {};

    bool gotNewData = false;

    bool imuIsLevel =
        fabsf(core->vstate.phi) < data->maxArmingAngle &&
        fabsf(core->vstate.theta) < data->maxArmingAngle;

    Receiver::poll(
            &data->rx,
            usec,
            imuIsLevel, 
            calibrating,
            &rxax,
            data->motorDevice,
            &data->arming,
            &pidItermResetReady,
            &pidItermResetValue,
            &gotNewData);

    if (pidItermResetReady) {
        core->pidReset = pidItermResetValue;
    }

    if (gotNewData) {
        memcpy(&data->rxAxes, &rxax, sizeof(Receiver::axes_t));
    }
}


class ReceiverTask : public Task {

    public:

        ReceiverTask()
            : Task(33) // Hz
        {
        }

        // Increase priority for RX task
        void adjustDynamicPriority(Task::data_t *td, uint32_t usec) 
        {
            if (m_dynamicPriority > 0) {
                m_ageCycles = 1 + (cmpTimeUs(usec,
                            m_lastSignaledAtUs) / m_desiredPeriodUs);
                m_dynamicPriority = 1 + m_ageCycles;
            } else  {
                if (Receiver::check(&td->rx, usec)) {
                    m_lastSignaledAtUs = usec;
                    m_ageCycles = 1;
                    m_dynamicPriority = 2;
                } else {
                    m_ageCycles = 0;
                }
            }
        }    
        
        void fun(
                HackflightCore::data_t * core,
                Task::data_t * data,
                uint32_t time)
        {
            bool calibrating = data->gyro.isCalibrating; 
            // || acc.calibrating != 0;
            bool pidItermResetReady = false;
            bool pidItermResetValue = false;

            Receiver::axes_t rxax = {};

            bool gotNewData = false;

            bool imuIsLevel =
                fabsf(core->vstate.phi) < data->maxArmingAngle &&
                fabsf(core->vstate.theta) < data->maxArmingAngle;

            Receiver::poll(
                    &data->rx,
                    time,
                    imuIsLevel, 
                    calibrating,
                    &rxax,
                    data->motorDevice,
                    &data->arming,
                    &pidItermResetReady,
                    &pidItermResetValue,
                    &gotNewData);

            if (pidItermResetReady) {
                core->pidReset = pidItermResetValue;
            }

            if (gotNewData) {
                memcpy(&data->rxAxes, &rxax, sizeof(Receiver::axes_t));
            }
        }
};
