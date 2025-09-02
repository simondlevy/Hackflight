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

#include <arduino_freertos.h>
#include <FreeRTOS.h>
#include <task.h>

using namespace arduino;

#include <__control__.hpp>

#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/led.hpp>
#include <tasks/logger.hpp>
#include <tasks/opticalflow.hpp>
#include <tasks/zranger.hpp>

#include <safety.hpp>
#include <uart.hpp>

static Motors motors;

static Safety safety = Safety(&motors);

static DebugTask debugTask;
static EstimatorTask estimatorTask;
static ImuTask imuTask;
static LedTask ledTask;
static LoggerTask loggerTask;
static OpticalFlowTask opticalFlowTask;
static ZRangerTask zrangerTask;

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

    ledTask.begin(&safety);

    estimatorTask.begin(&safety);

    loggerTask.begin(&estimatorTask, &closedLoopControl);

    imuTask.begin(&estimatorTask, &debugTask);

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
