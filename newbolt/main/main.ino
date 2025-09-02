#include <STM32FreeRTOS.h>

#include "task_debug.hpp"
#include "task_estimator.hpp"
#include "task_imu.hpp"
#include "task_led.hpp"
#include "task_opticalflow.hpp"
#include "task_zranger.hpp"

#include "safety.hpp"

static const uint8_t LED_PIN = PC0;

static Motors motors;

static Safety safety = Safety(&motors);

static DebugTask debugTask;
static EstimatorTask estimatorTask;
static ImuTask imuTask;
static LedTask ledTask;
static OpticalFlowTask opticalFlowTask;
static ZRangerTask zrangerTask;

void setup() 
{
    Serial.begin(115200);

	debugTask.begin();

    zrangerTask.begin(&estimatorTask);

    opticalFlowTask.begin(&estimatorTask);

    ledTask.begin(&safety, LED_PIN, true);

    estimatorTask.begin(&safety);

    vTaskStartScheduler();

    Serial.println("Insufficient RAM");

    while (true);
}

void loop() 
{
}
