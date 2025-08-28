#include <SPI.h>

#include <arduino_freertos.h>
#include <FreeRTOS.h>
#include <task.h>

using namespace arduino;

#define M2T(X) ((unsigned int)(X))
#define T2M(X) ((unsigned int)(X))

#include <hackflight.h>

// Chosen at config time
#include "__control__.hpp"

#include <safety.hpp>
#include <tasks/debug.hpp>
#include <tasks/flowdeck.hpp>
#include <tasks/led.hpp>
#include <tasks/comms/logging.hpp>
#include <tasks/comms/setpoint.hpp>
#include <tasks/zranger.hpp>

static Safety safety;

static DebugTask debugTask;
static EstimatorTask estimatorTask;
static LedTask ledTask;
static SetpointTask setpointTask;
static ZRangerTask zrangerTask;
static FlowDeckTask flowDeckTask;

static const uint8_t FLOWDECK_CS_PIN = 3;

static const uint8_t LED_PIN = 15;

static HardwareSerial * uart = &Serial5;

void setup() 
{
    Serial.begin(0);

    uart->begin(115200);

    SPI.begin();

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

	debugTask.begin();

    zrangerTask.begin(&estimatorTask, &debugTask);

    flowDeckTask.begin(&estimatorTask, FLOWDECK_CS_PIN, &debugTask);

    estimatorTask.begin(&safety);

    setpointTask.begin(&safety, uart);

    ledTask.begin(&safety, LED_PIN);

    vTaskStartScheduler();
}

void loop() {}
