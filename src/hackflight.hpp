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
#include <pidcontrol.hpp>
#include <mixers/crazyflie.hpp>
#include <tasks/core.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/led.hpp>
#include <tasks/logging.hpp>
#include <tasks/setpoint.hpp>
#include <tasks/task2.hpp>

class Hackflight {

    public:

        void init()
        {
            Comms::init();

            task2.begin(&estimatorTask);

            estimatorTask.begin();

            setpointTask.begin();

            loggingTask.begin(&estimatorTask, &pidControl);

            ledTask.begin(&imuTask);

            imuTask.begin(&estimatorTask);

            coreTask.begin(
                    &pidControl,
                    &estimatorTask,
                    &imuTask,
                    &ledTask,
                    &setpointTask,
                    Mixer::rotorCount,
                    Mixer::mix);
        }


    private:

        CoreTask coreTask;
        EstimatorTask estimatorTask;
        LedTask ledTask;
        ImuTask imuTask;
        LoggingTask loggingTask;
        SetpointTask setpointTask;
        Task2 task2;

        PidControl pidControl;
};
