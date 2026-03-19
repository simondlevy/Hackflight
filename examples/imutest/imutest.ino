#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/filters/imufilter.hpp>
#include <firmware/imus/lsm6dso.hpp>
//#include <firmware/imus/mpu6050.hpp>

static hf::IMU _imu;

static hf::ImuFilter _imuFilter;

void setup()
{
    Serial.begin(0);

    _imu.begin();
}

void loop() 
{
    const auto imuraw = _imu.read();

    _imuFilter = hf::ImuFilter::step(_imuFilter, millis(), imuraw,
            _imu.gyroRangeDps(), _imu.accelRangeGs());

    hf::Debugger::report(_imuFilter.output);
}
