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

#include <tasks/comms/logging.hpp>
#include <tasks/comms/setpoint.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/led.hpp>
#include <tasks/opticalflow.hpp>
#include <tasks/zranger.hpp>

static const float IMU_CALIBRATION_PITCH = 0;
static const float IMU_CALIBRATION_ROLL = 0;

static ClosedLoopControl closedLoopControl;

static Safety safety;

static DebugTask debugTask;
static EstimatorTask estimatorTask;
static OpticalFlowTask opticalFlowTask;
static ImuTask imuTask;
static LedTask ledTask;
static LoggerTask loggerTask;
static SetpointTask setpointTask;
static ZRangerTask zrangerTask;

static const uint8_t FLOW_CS_PIN = 3;

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

    opticalFlowTask.begin(&estimatorTask, FLOW_CS_PIN, &debugTask);

    estimatorTask.begin(&safety);

    setpointTask.begin(&safety, uart);

    loggerTask.begin(&estimatorTask, &closedLoopControl, uart);

    ledTask.begin(&safety, LED_PIN);

    imuTask.begin(&estimatorTask, &debugTask,
            IMU_CALIBRATION_ROLL, IMU_CALIBRATION_PITCH);

    vTaskStartScheduler();
}

void loop() {}
