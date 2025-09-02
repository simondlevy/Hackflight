#include <STM32FreeRTOS.h>

#include "task.hpp"

static constexpr float HEARTBEAT_HZ = 1;

static constexpr uint32_t PULSE_MSEC = 50;

static uint32_t m2t(const uint32_t msec)
{
    return (200L * configTICK_RATE_HZ) / 1000L;
}

static const uint8_t LED_PIN = PC0;

static const size_t STACKSIZE = 3 * configMINIMAL_STACK_SIZE; // arbitrary

static StackType_t  _taskStackBuffer[STACKSIZE]; 

static StaticTask_t _taskTaskBuffer;

void set(const bool on)
{
    digitalWrite(LED_PIN, !on);
}

static void taskfun(void* arg) {

    (void)arg;

    pinMode(LED_PIN, OUTPUT);

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {

        set(true);
        vTaskDelay(PULSE_MSEC);
        set(false);
        vTaskDelayUntil(&lastWakeTime, 1000/HEARTBEAT_HZ);
    }
}

void setup() 
{
    Serial.begin(115200);

    xTaskCreateStatic(
            taskfun,
            "task", // name
            STACKSIZE,
            NULL,   // obj
            1,      // priority
            _taskStackBuffer,
            &_taskTaskBuffer);

    vTaskStartScheduler();

    Serial.println("Insufficient RAM");

    while (true);
}

void loop() 
{
}
