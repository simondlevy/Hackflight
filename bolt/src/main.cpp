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

#define __main
#include <Arduino.h>

#include <stm32fxxx.h>

#include <free_rtos/FreeRTOS.h>
#include <free_rtos/semphr.h>
#include <free_rtos/task.h>

// Chosen at config time
#include <__control__.hpp>

#include <hackflight.h>
#include <mixers/crazyflie.hpp>
#include <safety.hpp>
#include <tasks/core.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/opticalflow.hpp>
#include <tasks/imu.hpp>
#include <tasks/led.hpp>
#include <tasks/logging.hpp>
#include <tasks/setpoint.hpp>
#include <tasks/zranger.hpp>
#include <uart_api.h>
#include <usb_api.h>

#include <hal/digital.h>
#include <hal/exti.h>
#include <hal/i2cdev.h>
#include <hal/nvic.h>
#include <hal/spi2.h>
#include <hal/hal_uart.h>
#include <hal/time.h>
#include <hal/usb.h>

#include <bosch/bmi088.h>
#include <bosch/bstdr_types.h>

#include <vl53l1.hpp>

#include <motors.h>

#include <st/vl53l1_api.h>

static const uint8_t OPTICALFLOW_CS_PIN = 11;

static const uint8_t LED_PIN = 4;

static CoreTask coreTask;
static DebugTask debugTask;
static EstimatorTask estimatorTask;
static OpticalFlowTask flowDeckTask;
static ImuTask imuTask;
static LedTask ledTask;
static LoggerTask loggerTask;
static SetpointTask setpointTask;
static ZRangerTask zrangerTask;

static ClosedLoopControl closedLoopControl;

static Safety safety;

static bool selftestPassed;

static bool didInit;

static void systemTask(void *arg)
{
    if (didInit) {
        return;
    }

    bool pass = true;

    didInit = true;

	debugTask.begin();
    
    zrangerTask.begin(&estimatorTask);

    flowDeckTask.begin(&estimatorTask, OPTICALFLOW_CS_PIN);

    estimatorTask.begin(&safety);

    setpointTask.begin(&safety);

    loggerTask.begin(&estimatorTask, &closedLoopControl);

    ledTask.begin(&safety, LED_PIN, true);

    imuTask.begin(&estimatorTask);

    auto coreTaskReady = coreTask.begin(
            &closedLoopControl,
            &safety,
            &estimatorTask,
            &imuTask,
            &setpointTask,
            Mixer::rotorCount,
            Mixer::mix);

    if (!coreTaskReady) {
        pass = false;
    }

    if (pass) {
        selftestPassed = true;
    }

    else {

        selftestPassed = false;

        if (didInit) {

            while (true) {

                //vTaskDelay(M2T(2000));
                delay(2000);

                if (selftestPassed)
                {
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

// IMU interrupt -------------------------------------------------------------

extern "C" {

    void __attribute__((used)) EXTI14_Callback(void) 
    {
        imuTask.dataAvailableCallback();
    }
}

// Main -----------------------------------------------------------------------

int main() 
{
    nvicInit();
    extiInit();
    usecTimerInit();
    i2cdevInit();
    usbInit();
    uartInit(115200); // Bluetooth comms

    SPI.begin();

    xTaskCreate(
            systemTask, 
            "SYSTEM",
            2* configMINIMAL_STACK_SIZE, 
            NULL, 
            2, 
            NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while(true) {
    }

    return 0;
}

