#include <STM32FreeRTOS.h>
#include <semphr.h>

#include <hackflight.h>
#include <motors.hpp>
#include <safety.hpp>

#include <tasks/debug.hpp>

const uint8_t LED_PIN = PC0;

static Motors motors;

static Safety safety = Safety(&motors);

static xSemaphoreHandle sem;

static void Thread1(void* arg) 
{
    (void)arg;

    while (true) {

        // Wait for signal from thread 2.
        xSemaphoreTake(sem, portMAX_DELAY);

        digitalWrite(LED_PIN, LOW);
    }
}

static void Thread2(void* arg) 
{
    (void)arg;

    pinMode(LED_PIN, OUTPUT);

    while (true) {

        digitalWrite(LED_PIN, HIGH);

        vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);

        xSemaphoreGive(sem);

        vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);
    }
}

void setup() 
{
    Serial.begin(115200);

    sem = xSemaphoreCreateCounting(1, 0);

    portBASE_TYPE s1 = xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    portBASE_TYPE s2 = xTaskCreate(Thread2, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    if (sem== NULL || s1 != pdPASS || s2 != pdPASS ) {
        while (true) {
            Serial.println("Problem creating tasks");
            delay(500);
        }
    }

    vTaskStartScheduler();
}

void loop() 
{
}
