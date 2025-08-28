#include <arduino_freertos.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

using namespace arduino;

//#include <hackflight.h>
//#include <tasks/led.hpp>

static SemaphoreHandle_t canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

static void systemTask(void*) 
{
    canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
    xSemaphoreTake(canStartMutex, portMAX_DELAY);

    pinMode(LED_BUILTIN, OUTPUT);

    while (true) {
        digitalWriteFast(LED_BUILTIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));

        digitalWriteFast(LED_BUILTIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    vTaskDelete(nullptr);
}

void setup() 
{
    Serial.begin(0);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    xTaskCreate(systemTask, "systemTask", 128, nullptr, 2, nullptr);

    vTaskStartScheduler();
}

void loop() {}
