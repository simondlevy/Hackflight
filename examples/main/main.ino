#include <arduino_freertos.h>
#include <FreeRTOS.h>
#include <task.h>

using namespace arduino;

#define M2T(X) ((unsigned int)(X))
#define T2M(X) ((unsigned int)(X))

#include <hackflight.h>

// Chosen at config time
#include <__control__.hpp>

#include <mixers/crazyflie.hpp>

#include <safety.hpp>

#include <tasks/comms/logging.hpp>
#include <tasks/comms/setpoint.hpp>
//#include <tasks/core.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/led.hpp>
#include <tasks/opticalflow.hpp>
#include <tasks/zranger.hpp>

static ClosedLoopControl closedLoopControl;

static Safety safety;

//static CoreTask coreTask;
static DebugTask debugTask;
static EstimatorTask estimatorTask;
static OpticalFlowTask opticalFlowTask;
static ImuTask imuTask;
static LedTask ledTask;
static LoggerTask loggerTask;
static SetpointTask setpointTask;
static ZRangerTask zrangerTask;

static HardwareSerial * uart = &Serial5;

void setup() 
{
    Serial.begin(0);

    uart->begin(115200);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

	debugTask.begin();

    zrangerTask.begin(&estimatorTask, &debugTask);

    opticalFlowTask.begin(&estimatorTask, &debugTask);

    estimatorTask.begin(&safety);

    setpointTask.begin(&safety, uart);

    loggerTask.begin(&estimatorTask, &closedLoopControl, uart);

    ledTask.begin(&safety);

    imuTask.begin(&estimatorTask, &debugTask);

    vTaskStartScheduler();
}

void loop() {}
