#include <arduino_freertos.h>
#include <FreeRTOS.h>
#include <task.h>

using namespace arduino;

#define M2T(X) ((unsigned int)(X))
#define T2M(X) ((unsigned int)(X))
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))

#include <hackflight.h>

// Chosen at config time
#include <__control__.hpp>

#include <mixers/crazyflie.hpp>

#include <safety.hpp>

#include <tasks/logging.hpp>
#include <tasks/setpoint.hpp>
#include <tasks/core.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/led.hpp>
#include <tasks/motors.hpp>
#include <tasks/opticalflow.hpp>
#include <tasks/zranger.hpp>

static const uint8_t LED_PIN = 15;

static const uint8_t OPTICALFLOW_CS_PIN = 3;

static ClosedLoopControl closedLoopControl;

static Safety safety;

static CoreTask coreTask;
static DebugTask debugTask;
static EstimatorTask estimatorTask;
static OpticalFlowTask opticalFlowTask;
static ImuTask imuTask;
static LedTask ledTask;
static LoggerTask loggerTask;
static MotorsTask motorsTask;
static SetpointTask setpointTask;
static ZRangerTask zrangerTask;

static HardwareSerial * uart = &Serial5;

static const uint8_t GYRO_INTERRUPT_PIN = 6;

static void handle_gyro_interrupt()
{
    imuTask.dataAvailableCallback();
}

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

    pinMode(GYRO_INTERRUPT_PIN, INPUT);
    attachInterrupt(GYRO_INTERRUPT_PIN, handle_gyro_interrupt, RISING);

	debugTask.begin();

    zrangerTask.begin(&estimatorTask);

    opticalFlowTask.begin(&estimatorTask, OPTICALFLOW_CS_PIN);

    estimatorTask.begin(&safety);

    setpointTask.begin(&safety);

    loggerTask.begin(&estimatorTask, &closedLoopControl);

    ledTask.begin(&safety, LED_PIN, false);

    imuTask.begin(&estimatorTask);

    coreTask.begin(
            &closedLoopControl,
            &safety,
            &estimatorTask,
            &imuTask,
            &setpointTask,
            Mixer::rotorCount,
            Mixer::mix,
            &debugTask);

    motorsTask.begin();

    vTaskStartScheduler();
}

void loop() {}
