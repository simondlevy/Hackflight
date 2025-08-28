#include <arduino_freertos.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

using namespace arduino;

#define M2T(X) ((unsigned int)(X))

#include <hackflight.h>
#include <safety.hpp>
#include <tasks/led.hpp>

//static SemaphoreHandle_t canStartMutex;
//static StaticSemaphore_t canStartMutexBuffer;

static Safety safety;

static LedTask ledTask;

void setup() 
{
    Serial.begin(0);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    ledTask.begin(&safety, LED_BUILTIN, false);

    vTaskStartScheduler();
}

void loop() {}
