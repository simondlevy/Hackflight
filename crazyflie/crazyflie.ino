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

#include <Wire.h>

#include <free_rtos_include.h>

#include <hackflight.h>
#include <comms.hpp>
#include <__control__.hpp>
#include <mixers/crazyflie.hpp>
#include <motors.hpp>
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

static ClosedLoopControl closedLoopControl;

static Motors motors;

static Safety safety = Safety(&motors);

static CoreTask coreTask;
static DebugTask debugTask;
static EstimatorTask estimatorTask;
static ImuTask imuTask;
static LedTask ledTask;
static LoggingTask loggingTask;
static OpticalFlowTask opticalFlowTask;
static SetpointTask setpointTask;
static ZRangerTask zrangerTask;

static void systemTask(void *arg)
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
            &motors,
            Mixer::rotorCount,
            Mixer::mix);

    while (true) {
        vTaskDelay(portMAX_DELAY);
    }
}

void setup() 
{
    Wire.begin();
    Wire.setClock(400000);
    delay(100);

    xTaskCreate(
            systemTask, 
            "SYSTEM",
            2* configMINIMAL_STACK_SIZE, 
            NULL, 
            2, 
            NULL);

    vTaskStartScheduler();
}

void loop() 
{
}
