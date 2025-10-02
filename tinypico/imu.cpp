#include <BMI088.h>

#include <tasks/imu.hpp>

static const uint8_t ACCEL_ADDR = 0x19;
static const uint8_t GYRO_ADDR = 0x69;

static Bmi088Accel _accel = Bmi088Accel(Wire, ACCEL_ADDR);

static Bmi088Gyro _gyro = Bmi088Gyro(Wire, GYRO_ADDR);

static bool failed(const int status)
{
    return status < 0;
}

bool ImuTask::device_init()
{
    Wire.begin();
    delay(100);

    if (failed(_accel.begin())) return false;

    if (failed(_gyro.begin())) return false;

    if (failed(_gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ))) return false;

    if (failed(_gyro.setRange(Bmi088Gyro::RANGE_2000DPS))) return false;

    if (failed(_gyro.pinModeInt3(
                    Bmi088Gyro::PIN_MODE_PUSH_PULL,
                    Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH))) return false;

    if (failed(_gyro.mapDrdyInt3(true))) return false;

    if (failed(_accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ))) return false;

    if (failed(_accel.setRange(Bmi088Accel::RANGE_24G))) return false;

    return true;
}

void ImuTask::device_readRaw(
        int16_t & gx, int16_t & gy, int16_t & gz,
        int16_t & ax, int16_t & ay, int16_t & az)
{
    _gyro.readSensor();

    gx = _gyro.getGyroX_raw();
    gy = _gyro.getGyroY_raw();
    gz = _gyro.getGyroZ_raw();

    _accel.readSensor();

    ax = _accel.getAccelX_raw();
    ay = _accel.getAccelY_raw();
    az = _accel.getAccelZ_raw();
}

float ImuTask::device_gyroRaw2Dps(const int16_t raw)
{
    return (float)raw * 2 * 2000 / 65536.f;
}

float ImuTask::device_accelRaw2Gs(const int16_t raw)
{
    return (float)raw * 2 * 24 / 65536.f;
}
