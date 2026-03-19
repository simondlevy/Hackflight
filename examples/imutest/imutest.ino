#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/ekf/ekf.hpp>
#include <firmware/filters/imufilter.hpp>
#include <firmware/imus/lsm6dso.hpp>
//#include <firmware/imus/mpu6050.hpp>

static hf::IMU _imu;

static hf::ImuFilter _imuFilter;

static hf::EKF _ekf;

void setup()
{
    Serial.begin(0);

    _imu.begin();
}

void loop() 
{
    const auto imuraw = _imu.read();

    hf::Debugger::report(imuraw);

    _imuFilter = hf::ImuFilter::step(_imuFilter, millis(), imuraw,
            _imu.gyroRangeDps(), _imu.accelRangeGs());

    _ekf.enqueueImu(_imuFilter.output);
    //const auto state = _ekf.getVehicleState(millis(), true);
    //hf::Debugger::report(state);
}
