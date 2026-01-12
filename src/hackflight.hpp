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
#include <led.hpp>
#include <pidcontrol.hpp>
#include <mixers/crazyflie.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/logging.hpp>
#include <tasks/setpoint.hpp>
#include <tasks/task1.hpp>
#include <tasks/task2.hpp>

class Hackflight {

    public:

        void init()
        {
            Comms::init();

            task2.begin(&estimatorTask);

            estimatorTask.begin();

            setpointTask.begin();

            loggingTask.begin(&estimatorTask);

            imuTask.begin(&estimatorTask);

            task1.begin(
                    &led,
                    &pidControl,
                    &estimatorTask,
                    &imuTask,
                    &setpointTask,
                    Mixer::rotorCount,
                    Mixer::mix);
        }


    private:

        Led led;
        Task1 task1;
        EstimatorTask estimatorTask;
        ImuTask imuTask;
        LoggingTask loggingTask;
        SetpointTask setpointTask;
        Task2 task2;

        PidControl pidControl;
};
