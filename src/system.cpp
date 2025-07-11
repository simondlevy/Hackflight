/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
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

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

// Chosen at config time
#include <__control__.hpp>

#include <hackflight.h>
#include <mixers/crazyflie.hpp>
#include <safety.hpp>
#include <system.h>
#include <tasks/core.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/flowdeck.hpp>
#include <tasks/imu.hpp>
#include <tasks/led.hpp>
#include <tasks/comms/setpoint.hpp>
#include <tasks/comms/logging.hpp>
#include <tasks/zranger.hpp>

static const float IMU_CALIBRATION_PITCH = 0;
static const float IMU_CALIBRATION_ROLL = 0;

static CoreTask coreTask;
static DebugTask debugTask;
static EstimatorTask estimatorTask;
static FlowDeckTask flowDeckTask;
static ImuTask imuTask;
static LedTask ledTask;
static LoggerTask loggerTask;
static SetpointTask setpointTask;
static ZRangerTask zrangerTask;

static ClosedLoopControl closedLoopControl;

static Safety safety;

static bool selftestPassed;
static bool didInit;

static SemaphoreHandle_t canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

static uint8_t _led_pin;
static uint8_t _flowdeck_cs_pin;

static void start()
{
    xSemaphoreGive(canStartMutex);
}

static void systemTask(void *arg)
{
    if (didInit) {
        return;
    }

    bool pass = true;

    canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
    xSemaphoreTake(canStartMutex, portMAX_DELAY);

    didInit = true;
    
	debugTask.begin();
    
    zrangerTask.begin(&estimatorTask);

    flowDeckTask.begin(&estimatorTask, _flowdeck_cs_pin);

    estimatorTask.begin(&safety);

    setpointTask.begin(&safety);

    loggerTask.begin(&estimatorTask, &closedLoopControl);

    ledTask.begin(&safety, _led_pin);

    imuTask.begin(
            &estimatorTask, 
            IMU_CALIBRATION_ROLL,
            IMU_CALIBRATION_PITCH);

    auto coreTaskReady = coreTask.begin(
            &closedLoopControl,
            &safety,
            &estimatorTask,
            &imuTask,
            &setpointTask,
            &debugTask,
            Mixer::rotorCount,
            Mixer::mix);

    if (!coreTaskReady) {
        pass = false;
    }

    if (pass) {
        selftestPassed = true;
        start();
    }

    else {

        selftestPassed = false;

        if (didInit) {

            while (true) {

                //vTaskDelay(M2T(2000));
                delay(2000);

                if (selftestPassed)
                {
                    start();
                    break;
                }
            }
        }
    }

    // Should never reach this point!
    while (true) {
        //vTaskDelay(portMAX_DELAY);
        delay(portMAX_DELAY);
    }
}


//////////////////////////////////////////////////////////////////////////////

void systemWaitStart(void)
{
    // This guarantees that the system task is initialized before other
    // tasks wait for the start event.
    while (!didInit) {
        //vTaskDelay(2);
        delay(2);
    }

    xSemaphoreTake(canStartMutex, portMAX_DELAY);
    xSemaphoreGive(canStartMutex);
}

void systemInit(const uint8_t led_pin, const uint8_t flowdeck_cs_pin)
{
    _led_pin = led_pin;

    _flowdeck_cs_pin = flowdeck_cs_pin;

    // Launch the system task that will initialize and start everything
    xTaskCreate(
            systemTask, 
            "SYSTEM",
            2* configMINIMAL_STACK_SIZE, 
            NULL, 
            2, 
            NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
}

void systemReportForever(const char * msg)
{
	debugTask.setMessage(msg);
}
