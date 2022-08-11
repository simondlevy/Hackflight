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

        void fun(hackflight_t * hf, uint32_t time)
        {
            bool calibrating = hf->gyro.isCalibrating; // || acc.calibrating != 0;
            bool pidItermResetReady = false;
            bool pidItermResetValue = false;

            rx_axes_t rxax = {};

            bool gotNewData = false;

            bool imuIsLevel =
                fabsf(hf->vstate.phi) < hf->maxArmingAngle &&
                fabsf(hf->vstate.theta) < hf->maxArmingAngle;

            rxPoll(
                    &hf->rx,
                    time,
                    imuIsLevel, 
                    calibrating,
                    &rxax,
                    hf->motorDevice,
                    &hf->arming,
                    &pidItermResetReady,
                    &pidItermResetValue,
                    &gotNewData);

            if (pidItermResetReady) {
                hf->pidReset = pidItermResetValue;
            }

            if (gotNewData) {
                memcpy(&hf->rxAxes, &rxax, sizeof(rx_axes_t));
            }
        }
};
