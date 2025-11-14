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

#define _MAIN

#include <comms.hpp>
#include <__control__.hpp>
#include <imu.hpp>
#include <mixers/crazyflie.hpp>
#include <tasks/core.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/logging.hpp>
#include <tasks/opticalflow.hpp>
#include <tasks/command.hpp>
#include <tasks/zranger.hpp>

class Hackflight {

    public:

        void init()
        {
            Comms::init();

            debugTask.begin();

            zrangerTask.begin(&estimatorTask);

            opticalFlowTask.begin(&estimatorTask);

            estimatorTask.begin();

            commandTask.begin();

            loggingTask.begin(&estimatorTask, &closedLoopControl);

            imu.begin(&estimatorTask);

            coreTask.begin(
                    &closedLoopControl,
                    &estimatorTask,
                    &imu,
                    &commandTask,
                    Mixer::rotorCount,
                    Mixer::mix);
        }


    private:

        CoreTask coreTask;
        DebugTask debugTask;
        EstimatorTask estimatorTask;
        Imu imu;
        LoggingTask loggingTask;
        OpticalFlowTask opticalFlowTask;
        CommandTask commandTask;
        ZRangerTask zrangerTask;

        ClosedLoopControl closedLoopControl;
};
