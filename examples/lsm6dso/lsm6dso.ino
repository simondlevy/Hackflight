#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/imus/lsm6dso.hpp>

static hf::IMU _imu;

void setup()
{
    Serial.begin(0);

    _imu.begin();
}

void loop() 
{
    hf::Debugger::report(_imu.read());
}
