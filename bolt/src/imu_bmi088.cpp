#include <tasks/imu.hpp>

float ImuTask::gyroRaw2Dps(const int16_t raw)
{
    return (float)raw * 2 * 2000 / 65536.f;
}

float ImuTask::accelRaw2Gs(const int16_t raw)
{
    return (float)raw * 2 * 24 / 65536.f;
}

const float ImuTask::rawGyroVarianceBase()
{
    return 100;
}
