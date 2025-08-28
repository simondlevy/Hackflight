/**
 *
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

#include <hackflight.h>
#include <free_rtos/FreeRTOS.h>
#include <free_rtos/semphr.h>
#include <free_rtos/task.h>
#include <safety.hpp>
#include <tasks/led.hpp>

static const uint8_t LED_PIN = PC0;

static bool didInit;

static SemaphoreHandle_t canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

static Safety safety;

static LedTask ledTask;

static void systemTask(void *arg)
{
    if (didInit) {
        return;
    }

    ledTask.begin(&safety, LED_PIN);

    canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
    xSemaphoreTake(canStartMutex, portMAX_DELAY);

    didInit = true;

    // Should never reach this point!
    while (true) {
        //vTaskDelay(portMAX_DELAY);
        delay(portMAX_DELAY);
    }
}


/*
#include <hackflight.h>
#include <system.h>
#include <Wire.h>

static const uint8_t FLOWDECK_CS_PIN = PB4;


void uartInit(const uint32_t baudrate);
*/

void setup() 
{
    Serial.begin(115200);

    // Launch the system task that will initialize and start everything
    xTaskCreate(
            systemTask, 
            "SYSTEM",
            2* configMINIMAL_STACK_SIZE, 
            NULL, 
            2, 
            NULL);

    // Start the FreeRTOS scheduler

     /*
    Wire.begin();
    Wire.setClock(400000);
    uartInit(115200);
    systemInit(LED_PIN, FLOWDECK_CS_PIN);
    */
}

void loop() 
{
}
