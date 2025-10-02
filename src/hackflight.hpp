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
#include <safety.hpp>
#include <tasks/core.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/led.hpp>
#include <tasks/logging.hpp>
#include <tasks/opticalflow.hpp>
#include <tasks/setpoint.hpp>
#include <tasks/zranger.hpp>

static CoreTask coreTask;
static DebugTask debugTask;
static EstimatorTask estimatorTask;
static LedTask ledTask;
static ImuTask imuTask;
static LoggingTask loggingTask;
static OpticalFlowTask opticalFlowTask;
static SetpointTask setpointTask;
static ZRangerTask zrangerTask;

static Safety safety;

static ClosedLoopControl closedLoopControl;

static void hackflight_init()
{
    Comms::init();

    debugTask.begin();

    zrangerTask.begin(&estimatorTask);

    opticalFlowTask.begin(&estimatorTask);

    estimatorTask.begin(&safety);

    setpointTask.begin(&safety);

    loggingTask.begin(&estimatorTask, &closedLoopControl);

    ledTask.begin(&safety, &imuTask);

    imuTask.begin(&estimatorTask);

    coreTask.begin(
            &closedLoopControl,
            &safety,
            &estimatorTask,
            &imuTask,
            &setpointTask,
            Mixer::rotorCount,
            Mixer::mix);
}
