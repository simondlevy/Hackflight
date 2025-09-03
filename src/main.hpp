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

#include <__control__.hpp>

#include <mixers/crazyflie.hpp>

#include <tasks/core.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/led.hpp>
#include <tasks/logger.hpp>
#include <tasks/motors.hpp>
#include <tasks/opticalflow.hpp>
#include <tasks/setpoint.hpp>
#include <tasks/zranger.hpp>

#include <safety.hpp>
#include <uart.hpp>

static CoreTask coreTask;
static DebugTask debugTask;
static EstimatorTask estimatorTask;
static ImuTask imuTask;
static LedTask ledTask;
static LoggerTask loggerTask;
static MotorsTask motorsTask;
static OpticalFlowTask opticalFlowTask;
static SetpointTask setpointTask;
static ZRangerTask zrangerTask;

static Safety safety = Safety(&motorsTask);

static ClosedLoopControl closedLoopControl;

static void handle_gyro_interrupt()
{
    imuTask.dataAvailableCallback();
}


void setup() 
{
    Serial.begin(115200);

    Uart::begin(115200);

	debugTask.begin();

    zrangerTask.begin(&estimatorTask);

    opticalFlowTask.begin(&estimatorTask);

    estimatorTask.begin(&safety);

    setpointTask.begin(&safety, &debugTask);

    loggerTask.begin(&estimatorTask, &closedLoopControl);

    ledTask.begin(&safety);

    imuTask.begin(&estimatorTask);

    motorsTask.begin(&debugTask);

    coreTask.begin(
            &closedLoopControl,
            &safety,
            &estimatorTask,
            &imuTask,
            &setpointTask,
            &motorsTask,
            Mixer::rotorCount,
            Mixer::mix,
            &debugTask);

    const uint8_t pin = imuTask.device_getInterruptPin();
    pinMode(pin, INPUT);
    attachInterrupt(pin, handle_gyro_interrupt, RISING);

    vTaskStartScheduler();

    Serial.println("Insufficient RAM");

    while (true);
}

void loop() 
{
}
