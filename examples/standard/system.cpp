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

#include <hackflight.h>
#include <mixers/crazyflie.hpp>

#include <safety.hpp>

#if 0

#include <tasks/core.hpp>
#include <tasks/estimator.hpp>
#include <tasks/flowdeck.hpp>
#include <tasks/imu.hpp>
#include <tasks/led.hpp>
#include <tasks/rpisetpoint.hpp>
#include <tasks/rpilogger.hpp>
#include <tasks/zranger.hpp>


static const float IMU_CALIBRATION_PITCH = 0;
static const float IMU_CALIBRATION_ROLL = 0;

// ---------------------------------------------------------------------------

static RpiSetpointTask rpiSetpointTask;
static RpiLoggerTask rpiLoggerTask;
static CoreTask coreTask;
static EstimatorTask estimatorTask;
static FlowDeckTask flowDeckTask;
static ImuTask imuTask;
static LedTask ledTask;
static ZRangerTask zrangerTask;

static Safety safety;

static bool selftestPassed;
static bool didInit;

static xSemaphoreHandle canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

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
    
    zrangerTask.begin(&estimatorTask);

    flowDeckTask.begin(&estimatorTask);

    estimatorTask.begin(&safety);

    rpiSetpointTask.begin(&safety);

    rpiLoggerTask.begin(&estimatorTask);

    ledTask.begin(&safety);

    imuTask.begin(
            &estimatorTask, 
            IMU_CALIBRATION_ROLL,
            IMU_CALIBRATION_PITCH);

    auto coreTaskReady = coreTask.begin(
            &safety,
            &estimatorTask,
            &imuTask,
            &rpiSetpointTask,
            Mixer::rotorCount,
            Mixer::mix);

    if (!coreTaskReady) {
        pass = false;
        error("SYSTEM: core task [FAIL]");
    }

    //Start the firmware
    if (pass) {
        selftestPassed = 1;
        start();
    }

    else {

        selftestPassed = 0;

        if (didInit) {

            while (true) {

                //vTaskDelay(M2T(2000));
                delay(2000);

                if (selftestPassed)
                {
                    debug("SYSTEM: Start forced");
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

void systemInit()
{

    void init_platform_specific();
    init_platform_specific();

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
#endif
