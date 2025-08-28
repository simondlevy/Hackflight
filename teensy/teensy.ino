#include <arduino_freertos.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

using namespace arduino;

#define M2T(X) ((unsigned int)(X))

#include <hackflight.h>
#include <safety.hpp>
#include <tasks/debug.hpp>
#include <tasks/led.hpp>

static SemaphoreHandle_t canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

static Safety safety;

static LedTask ledTask;
static DebugTask debugTask;

static bool didInit;

void systemWaitStart(void)
{
    // This guarantees that the system task is initialized before other
    // tasks wait for the start event.
    while (!didInit) {
        delay(2);
    }

    xSemaphoreTake(canStartMutex, portMAX_DELAY);
    xSemaphoreGive(canStartMutex);
}


void setup() 
{
    Serial.begin(0);

    canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
    xSemaphoreTake(canStartMutex, portMAX_DELAY);

    didInit = true;

	debugTask.begin();

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    ledTask.begin(&safety, LED_BUILTIN, false);

    vTaskStartScheduler();
}

void loop() {}
