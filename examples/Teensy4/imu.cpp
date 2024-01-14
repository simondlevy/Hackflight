#include <Wire.h>

#include <usfs.hpp>

#include <tasks/imu.hpp>

static const uint8_t INTERRUPT_PIN = 12; 

static const uint8_t ACCEL_BANDWIDTH = 3;
static const uint8_t GYRO_BANDWIDTH  = 3;
static const uint8_t QUAT_DIVISOR    = 1;
static const uint8_t MAG_RATE        = 100;
static const uint8_t ACCEL_RATE      = 20; // Multiply by 10 to get actual rate
static const uint8_t GYRO_RATE       = 100; // Multiply by 10 to get actual rate
static const uint8_t BARO_RATE       = 50;

static const bool VERBOSE = false;

static const uint8_t INTERRUPT_ENABLE = 
Usfs::INTERRUPT_RESET_REQUIRED |
Usfs::INTERRUPT_ERROR |
Usfs::INTERRUPT_QUAT;

static Usfs usfs;

static volatile bool _gotNewData;

static ImuTask * _imuTask;

static void interruptHandler()
{
    _imuTask->dataAvailableCallback();

    _gotNewData = true;
}

void ImuTask::deviceInit(void)
{
    /*
    _imuTask = this;

    usfs.loadFirmware(VERBOSE); 

    usfs.begin(
            ACCEL_BANDWIDTH,
            GYRO_BANDWIDTH,
            QUAT_DIVISOR,
            MAG_RATE,
            ACCEL_RATE,
            GYRO_RATE,
            BARO_RATE,
            INTERRUPT_ENABLE,
            VERBOSE); 

    pinMode(INTERRUPT_PIN, INPUT);

    attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);  

    // Clear interrupts
    Usfs::checkStatus();*/
}

void ImuTask::readGyro(Axis3i16* dataOut)
{
    /*
    if (Usfs::eventStatusIsGyrometer(Usfs::checkStatus())) {

        usfs.readGyrometerRaw((int16_t *)dataOut);
    }*/
}

void ImuTask::readAccel(Axis3i16* dataOut)
{
    /*
    if (Usfs::eventStatusIsAccelerometer(Usfs::checkStatus())) {

        usfs.readAccelerometerRaw((int16_t *)dataOut);
    }*/
}
