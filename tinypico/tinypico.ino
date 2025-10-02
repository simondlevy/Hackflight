/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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




#include <hackflight.h>
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

void setup() 
{
    Comms::init();

    estimatorTask.begin(&safety);

    debugTask.begin();

    imuTask.begin(&estimatorTask);

    ledTask.begin(&safety, &imuTask);

    loggingTask.begin(&estimatorTask, &closedLoopControl);

    setpointTask.begin(&safety);

    opticalFlowTask.begin(&estimatorTask);

    zrangerTask.begin(&estimatorTask);

    coreTask.begin(
            &closedLoopControl,
            &safety,
            &estimatorTask,
            &imuTask,
            &setpointTask,
            Mixer::rotorCount,
            Mixer::mix,
            &debugTask);
}

void loop()
{
}
