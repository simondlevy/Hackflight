#include <arduino_freertos.h>

static void systemTask(void*) 
{
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);

    while (true) {
        digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
        vTaskDelay(pdMS_TO_TICKS(500));

        digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
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
