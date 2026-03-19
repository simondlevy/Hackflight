#include <LSM6DSOSensor.h>

#include <hackflight.h>
#include <firmware/debugging.hpp>

static LSM6DSOSensor AccGyr(&Wire);

void setup()
{
    Serial.begin(0);

    Wire.begin();

    AccGyr.begin();
    AccGyr.Enable_X();
    AccGyr.Enable_G();
}

void loop() 
{
    int32_t accel[3] = {};
    AccGyr.Get_X_Axes(accel);

    int32_t gyro[3] = {};
    AccGyr.Get_G_Axes(gyro);

    const auto imuraw = hf::ImuRaw(
            hf::Vec3Raw(gyro[0], gyro[1], gyro[2]),
            hf::Vec3Raw(accel[0], accel[1], accel[2]));

    hf::Debugger::report(imuraw);
}
