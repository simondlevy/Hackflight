#include <Wire.h>

#include <arduino_freertos.h>
#include <FreeRTOS.h>
#include <task.h>

using namespace arduino;

#define M2T(X) ((unsigned int)(X))
#define T2M(X) ((unsigned int)(X))

#include <hackflight.h>
#include <safety.hpp>
#include <tasks/debug.hpp>
#include <tasks/led.hpp>
#include <tasks/zranger.hpp>

static Safety safety;

static DebugTask debugTask;
static EstimatorTask estimatorTask;
static LedTask ledTask;
static ZRangerTask zrangerTask;

void setup() 
{
    Wire.begin();
    Wire.setClock(400000);

	debugTask.begin();

    zrangerTask.begin(&estimatorTask, &debugTask);

    debugTask.setMessage("hello");

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    ledTask.begin(&safety, 15, false);

    vTaskStartScheduler();
}

void loop() {}
