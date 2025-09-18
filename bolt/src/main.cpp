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

#include <hackflight.h>
#include <mixers/crazyflie.hpp>
#include <motors.hpp>
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

#include <hal/exti.h>
#include <hal/i2cdev.h>
#include <hal/nvic.h>
#include <hal/hal_uart.h>
#include <hal/usb.h>

static const uint8_t OPTICALFLOW_CS_PIN = 11;

static const uint8_t LED_PIN = 4;

static CoreTask coreTask;
static DebugTask debugTask;
static EstimatorTask estimatorTask;
static OpticalFlowTask opticalFlowTask;
static ImuTask imuTask;
static LedTask ledTask;
static LoggerTask loggerTask;
static SetpointTask setpointTask;
static ZRangerTask zrangerTask;

static ClosedLoopControl closedLoopControl;

static Motors motors;

static Safety safety = Safety(&motors);

static void systemTask(void *arg)
{
	debugTask.begin();
    
    zrangerTask.begin(&estimatorTask);

    opticalFlowTask.begin(&estimatorTask, OPTICALFLOW_CS_PIN);

    estimatorTask.begin(&safety);

    setpointTask.begin(&safety);

    loggerTask.begin(&estimatorTask, &closedLoopControl);

    ledTask.begin(&safety, LED_PIN, true);

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

// IMU interrupt -------------------------------------------------------------

extern "C" {

    void __attribute__((used)) EXTI14_Callback(void) 
    {
        imuTask.dataAvailableCallback();
    }
}

// Stop motors on system fault ------------------------------------------------

extern "C" {
    void stop_motors()
    {
        motors.stop();
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

    return 0;
}

