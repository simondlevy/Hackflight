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
#include <debugger.hpp>
#include <imu.hpp>
#include <mixers/crazyflie.hpp>
#include <tasks/core.hpp>
#include <tasks/estimator.hpp>
#include <tasks/opticalflow.hpp>
#include <tasks/zranger.hpp>

class Hackflight {

    public:

        void init()
        {
            Comms::init();

            zrangerTask.begin(&estimatorTask);

            opticalFlowTask.begin(&estimatorTask);

            estimatorTask.begin();

            imu.begin(&estimatorTask);

            coreTask.begin(
                    &closedLoopControl,
                    &estimatorTask,
                    &imu,
                    Mixer::rotorCount,
                    Mixer::mix);
        }

    private:


        CoreTask coreTask;
        EstimatorTask estimatorTask;
        Imu imu;
        OpticalFlowTask opticalFlowTask;
        ZRangerTask zrangerTask;

        ClosedLoopControl closedLoopControl;
        Debugger debugger;
};
