#include <Wire.h>

#include <BMI088.h>

#include <hackflight.h>
#include <firmware/debugger.hpp>

static auto imu_ = Bmi088(Wire, 0x18, 0x68);

static auto ImuOkay(const int status) -> bool
{
    return status >= 0;
}


void setup()
{
    if (!(ImuOkay(imu_.begin()) &&

          imu_.setOdr(Bmi088::ODR_1000HZ)

          && imu_.setRange(
              Bmi088::ACCEL_RANGE_24G,  Bmi088::GYRO_RANGE_2000DPS)

          && imu_.pinModeDrdy(
              Bmi088::PIN_MODE_PUSH_PULL, Bmi088::PIN_LEVEL_ACTIVE_HIGH)

          && imu_.mapSync(Bmi088::PIN_3)

          && imu_.mapDrdy(Bmi088::PIN_1))) {

        hf::Debugger::ReportForever("Unable to initialize IMU");
    }
}

void loop()
{
    static uint32_t count_;


}
