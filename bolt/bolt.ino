#include <STM32FreeRTOS.h>
#include <semphr.h>

#include <hackflight.h>
#include <motors.hpp>
#include <safety.hpp>

#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/led.hpp>

const uint8_t LED_PIN = PC0;

static Motors motors;

static Safety safety = Safety(&motors);

static DebugTask debugTask;
static EstimatorTask estimatorTask;
static LedTask ledTask;

static void systemTask(void *arg)
{
	debugTask.begin();

    /*
    zrangerTask.begin(&estimatorTask);

    opticalFlowTask.begin(&estimatorTask, OPTICALFLOW_CS_PIN);
    */

    estimatorTask.begin(&safety);

    /*
    setpointTask.begin(&safety);

    loggerTask.begin(&estimatorTask, &closedLoopControl);
    */

    ledTask.begin(&safety, LED_PIN, true);

    DebugTask::setMessage(&debugTask, "okay");

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
