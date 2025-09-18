#include <STM32FreeRTOS.h>
#include <semphr.h>

#include <hackflight.h>
#include <motors.hpp>
#include <safety.hpp>

#include <tasks/debug.hpp>
#include <tasks/led.hpp>

const uint8_t LED_PIN = PC0;

static Motors motors;

static Safety safety = Safety(&motors);

static DebugTask debugTask;
static LedTask ledTask;

/*
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
}*/

static void systemTask(void *arg)
{
	debugTask.begin();

    /*
    zrangerTask.begin(&estimatorTask);

    opticalFlowTask.begin(&estimatorTask, OPTICALFLOW_CS_PIN);

    estimatorTask.begin(&safety);

    setpointTask.begin(&safety);

    loggerTask.begin(&estimatorTask, &closedLoopControl);
    */

    ledTask.begin(&safety, LED_PIN, true);

    /*
    imuTask.begin(&estimatorTask);

    coreTask.begin(
            &closedLoopControl,
            &safety,
            &estimatorTask,
            &imuTask,
            &setpointTask,
            &motors,
            Mixer::rotorCount,
            Mixer::mix);
            */

    while (true) {
        vTaskDelay(portMAX_DELAY);
    }
}

void setup() 
{
    Serial.begin(115200);

    /*
    debugTask.begin();

    ledTask.begin(&safety, LED_PIN, true);

    DebugTask::setMessage(&debugTask, "okay");*/

    /*
    sem = xSemaphoreCreateCounting(1, 0);

    portBASE_TYPE s1 =
        xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    portBASE_TYPE s2 =
        xTaskCreate(Thread2, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    if (sem== NULL || s1 != pdPASS || s2 != pdPASS ) {
        while (true) {
            Serial.println("Problem creating tasks");
            delay(500);
        }
    }*/

    xTaskCreate(
            systemTask, 
            "SYSTEM",
            2* configMINIMAL_STACK_SIZE, 
            NULL, 
            2, 
            NULL);

    vTaskStartScheduler();
}

void loop() 
{
}
