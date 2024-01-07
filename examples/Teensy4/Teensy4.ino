#include <FreeRTOS_TEENSY4.h>

static void task1(void*) 
{

    while (true) {

        Serial.println("BADDA BOOM");
        digitalWriteFast(LED_BUILTIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));

        Serial.println("BADDA BING");
        digitalWriteFast(LED_BUILTIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void task2(void*) 
{

    while (true) {

        Serial.println("TICK");
        vTaskDelay(pdMS_TO_TICKS(1'000));

        Serial.println("TOCK");
        vTaskDelay(pdMS_TO_TICKS(1'000));
    }
}

FLASHMEM __attribute__((noinline)) void setup() {
    Serial.begin(115'200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWriteFast(LED_BUILTIN, HIGH);

    delay(5'000);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    xTaskCreate(task1, "task1", 128, nullptr, 2, nullptr);
    xTaskCreate(task2, "task2", 128, nullptr, 2, nullptr);

    Serial.println("setup(): starting scheduler...");
    Serial.flush();

    vTaskStartScheduler();
}

void loop() {}
