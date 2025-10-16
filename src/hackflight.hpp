/**
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include <comms.hpp>
#include <__control__.hpp>
#include <mixers/crazyflie.hpp>
#include <tasks/core.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/led.hpp>
#include <tasks/logging.hpp>
#include <tasks/opticalflow.hpp>
#include <tasks/setpoint.hpp>
#include <tasks/zranger.hpp>

class Hackflight {

    public:

        void init()
        {
            //Comms::init();

            debugTask.begin();

            // zrangerTask.begin(&estimatorTask, &debugTask);

            opticalFlowTask.begin(&estimatorTask, &debugTask);

            // estimatorTask.begin();

            // setpointTask.begin();

            // loggingTask.begin(&estimatorTask, &closedLoopControl);

            ledTask.begin(&imuTask);

            imuTask.begin(&estimatorTask);

            /*
            coreTask.begin(
                    &closedLoopControl,
                    &estimatorTask,
                    &imuTask,
                    &ledTask,
                    &setpointTask,
                    Mixer::rotorCount,
                    Mixer::mix);*/
        }


    private:

        CoreTask coreTask;
        DebugTask debugTask;
        EstimatorTask estimatorTask;
        LedTask ledTask;
        ImuTask imuTask;
        LoggingTask loggingTask;
        OpticalFlowTask opticalFlowTask;
        SetpointTask setpointTask;
        ZRangerTask zrangerTask;

        ClosedLoopControl closedLoopControl;
};
