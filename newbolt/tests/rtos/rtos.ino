#include <STM32FreeRTOS.h>

static uint32_t m2t(const uint32_t msec)
{
    return (200L * configTICK_RATE_HZ) / 1000L;
}

static const uint8_t LED_PIN = PC0;

SemaphoreHandle_t sem;

static void Task1(void* arg) 
{
    (void)arg;

    while (true) {

        xSemaphoreTake(sem, portMAX_DELAY);

        //digitalWrite(LED_PIN, LOW);
    }
}

static void Task2(void* arg) {
    
    (void)arg;

    pinMode(LED_PIN, OUTPUT);

    while (true) {

        digitalWrite(LED_PIN, HIGH);

        vTaskDelay(m2t(200));

        xSemaphoreGive(sem);

        digitalWrite(LED_PIN, LOW);

        vTaskDelay(m2t(200));

    }
}

void setup() 
{
    portBASE_TYPE s1, s2;

    Serial.begin(115200);

    sem = xSemaphoreCreateCounting(1, 0);

    s1 = xTaskCreate(Task1, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    s2 = xTaskCreate(Task2, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    if (sem== NULL || s1 != pdPASS || s2 != pdPASS ) {
        Serial.println(F("Creation problem"));
        while (true);
    }

    vTaskStartScheduler();
    Serial.println("Insufficient RAM");
    while (true);
}

void loop() 
{
}
