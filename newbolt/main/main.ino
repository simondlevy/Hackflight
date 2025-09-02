#include <STM32FreeRTOS.h>

static uint32_t m2t(const uint32_t msec)
{
    return (200L * configTICK_RATE_HZ) / 1000L;
}

static const uint8_t LED_PIN = PC0;

static const size_t STACKSIZE = 3 * configMINIMAL_STACK_SIZE; // arbitrary

static StackType_t  _taskStackBuffer[STACKSIZE]; 

static StaticTask_t _taskTaskBuffer;

static void taskfun(void* arg) {

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

    /*
    xTaskCreate(
            taskfun,
            "task", // name
            STACKSIZE,
            NULL,   // obj
            1,      // priority
            NULL);*/

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
