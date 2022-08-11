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
#include "rx.h"
#include "task.h"

class ReceiverTask : public Task {

    public:

        ReceiverTask()
            : Task(33) // Hz
        {
        }

        void fun(task_data_t * data, uint32_t time)
        {
            bool calibrating = data->gyro.isCalibrating; // || acc.calibrating != 0;
            bool pidItermResetReady = false;
            bool pidItermResetValue = false;

            rx_axes_t rxax = {};

            bool gotNewData = false;

            bool imuIsLevel =
                fabsf(data->vstate.phi) < data->maxArmingAngle &&
                fabsf(data->vstate.theta) < data->maxArmingAngle;

            rxPoll(
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
                data->pidReset = pidItermResetValue;
            }

            if (gotNewData) {
                memcpy(&data->rxAxes, &rxax, sizeof(rx_axes_t));
            }
        }
};
