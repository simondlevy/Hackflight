#include <LSM6DSOSensor.h>

#include <hackflight.h>
#include <firmware/debugging.hpp>

static LSM6DSOSensor _lsm6dso(&Wire);

static bool bad(const LSM6DSOStatusTypeDef status)
{
    return status != LSM6DSO_OK;
}

void setup()
{
    Serial.begin(0);

    Wire.begin();

    if (
            bad(_lsm6dso.begin())  ||
            bad(_lsm6dso.Enable_X())  ||
            bad(_lsm6dso.Enable_G())  ||
            bad(_lsm6dso.Set_X_FS(16)) ||
            bad(_lsm6dso.Set_G_FS(2000))) {

        hf::Debugger::reportForever(
                "LSM6DSO initialization unsuccessful\n");
    }
}

void loop() 
{
    int32_t accel[3] = {};
    _lsm6dso.Get_X_Axes(accel);

    int32_t gyro[3] = {};
    _lsm6dso.Get_G_Axes(gyro);

    const auto imuraw = hf::ImuRaw(
            hf::Vec3Raw(gyro[0], gyro[1], gyro[2]),
            hf::Vec3Raw(accel[0], accel[1], accel[2]));

    hf::Debugger::report(imuraw);
}
