#include <STM32FreeRTOS.h>

static uint32_t m2t(const uint32_t msec)
{
    return (200L * configTICK_RATE_HZ) / 1000L;
}

static const uint8_t LED_PIN = PC0;

static void Task2(void* arg) {
    
    (void)arg;

    pinMode(LED_PIN, OUTPUT);

    while (true) {

        digitalWrite(LED_PIN, HIGH);

        vTaskDelay(m2t(200));

        digitalWrite(LED_PIN, LOW);

        vTaskDelay(m2t(200));

    }
}

void setup() 
{
    Serial.begin(115200);

    xTaskCreate(Task2, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();

    Serial.println("Insufficient RAM");

    while (true);
}

void loop() 
{
}
